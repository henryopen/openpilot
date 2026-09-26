# ESP32-C3 rear-LED relay

```
PiBar (led_agent / led_card.Relay) --USB serial--> ESP32-C3 --2.4 GHz WiFi--> BX-Y04 card 192.168.22.1
```

The BX-Y04 controller only speaks 2.4 GHz, and in the car the PI's own WiFi holds the car's 5 GHz
hotspot. Two USB WiFi sticks were tried on 2026-09-26: a D-Link DWA-171 rev A1 (RTL8811AU) sees no
2.4 GHz under Linux at all, and a TP-Link TL-WN8200ND V2 trips the PI 5's 600 mA USB limit. The
ESP32-C3's USB is a fixed USB-Serial/JTAG block - it cannot be a USB network adapter - so this board
is a request relay instead: it joins the card's AP and forwards the PI's HTTP requests to it.

Written for the purpose on 2026-09-26 (by the findmy session on `.96`, from the brief in
`F:\claude\findmy\ESP32_LED_RELAY_任務書.md`), on a spare findmy ESP32-C3-MINI-1 board.

## Protocol (big-endian lengths)

```
PI  -> ESP  "LRQ1" method(1: 'P' POST, 'G' GET, 'I' info) pathLen(2) bodyLen(4) path body
ESP -> PI   "LRS1" status(2, int16: HTTP code, or <0 relay error) bodyLen(4) body
```
`I` answers `{"wifi":bool,"ip":..,"rssi":..,"heap":..,"up_ms":..}`. Errors: -1 not on the AP,
-2 frame too large, -3 no memory, -5 HTTP failed. Both USB-Serial/JTAG and UART0 (921600) are
served; nothing else is ever written to them (console and logs are off), except the ROM's boot
banner, which is why both ends resync on the magic.

## Behaviour to know

- **Opening the port on the PI resets the board** even with DTR/RTS held low (cdc_acm raises the
  lines on open): `up_ms` reads ~67 right after, and it rejoins the AP in ~2 s. It comes back
  running, not in the bootloader. `led_card.Relay` keeps the port open for the process lifetime.
- Measured on the PiBar 2026-09-26, board `303a:1001` on `/dev/ttyACM0`, RSSI -34:
  info round trip ~1 ms; card login 108-117 ms; `UpdateDynamic` per frame median 197 ms,
  p95 236 ms; 5-minute soak at led_agent's pace, 1497 frames, 0 failures (the card is the
  bottleneck, ~155 ms of each frame is the card rebuilding the area).
- No USB over-current on the PI with this board.

## Build and flash (on `.96`, PlatformIO + ESP-IDF 5.5 already installed for findmy)

```
copy src\wifi_secrets.h.example src\wifi_secrets.h   # and fill in the card's AP
pio run
python -m esptool --chip esp32c3 --port COMx write_flash 0x0 .pio\build\esp32c3\bootloader.bin ^
       0x8000 .pio\build\esp32c3\partitions.bin 0x10000 .pio\build\esp32c3\firmware.bin
python acceptance_test.py COMx        # info, verification code, login - must all be 200
```
The acceptance test only calls info / getVerificationCode / userLogin - never anything that
changes the card's settings (the 2026-09-23 mess started from changing them).
