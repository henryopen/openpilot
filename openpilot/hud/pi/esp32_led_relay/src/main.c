// ESP32-C3 LED relay
// Pi (led_agent) --USB serial--> ESP32-C3 --2.4G WiFi--> BX-Y04 control card (192.168.22.1)
//
// Request  (Pi -> ESP32): "LRQ1" | method(1) | pathLen(2) | bodyLen(4) | path | body   (big-endian)
// Response (ESP32 -> Pi): "LRS1" | status(int16) | bodyLen(4) | body
// method: 'P' POST, 'G' GET, 'I' status.  Both UART0 and USB-Serial/JTAG are served;
// the reply goes back on the transport the request came from.

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

// WIFI_SSID / WIFI_PASS: the card's own AP. Kept out of this public repo - copy
// wifi_secrets.h.example to wifi_secrets.h and fill it in before building.
#include "wifi_secrets.h"
#define CARD_IP         "192.168.22.1"
#define CARD_PORT       80

#define MAX_PATH        512
#define MAX_BODY        60000
#define RX_BUF_SIZE     (16 * 1024)
#define TX_BUF_SIZE     (4 * 1024)
#define UART_BAUD       921600
#define UART_TX_PIN     21
#define UART_RX_PIN     20

#define FRAME_TIMEOUT_MS    3000
#define CONNECT_TIMEOUT_MS  3000
#define HTTP_TIMEOUT_MS     10000

#define ST_NO_WIFI      (-1)
#define ST_BAD_REQ      (-2)
#define ST_NO_MEM       (-3)
#define ST_HTTP_FAIL    (-5)

static SemaphoreHandle_t s_req_lock;
static volatile bool s_wifi_up;
static char s_ip[16] = "";

// ---------------------------------------------------------------- transports

typedef struct {
    const char *name;
    int (*read)(uint8_t *buf, size_t len, TickType_t ticks);
    int (*write)(const uint8_t *buf, size_t len, TickType_t ticks);
} transport_t;

static int uart_rd(uint8_t *buf, size_t len, TickType_t ticks)
{
    return uart_read_bytes(UART_NUM_0, buf, len, ticks);
}

static int uart_wr(const uint8_t *buf, size_t len, TickType_t ticks)
{
    (void)ticks;
    return uart_write_bytes(UART_NUM_0, buf, len);
}

static int usj_rd(uint8_t *buf, size_t len, TickType_t ticks)
{
    return usb_serial_jtag_read_bytes(buf, len, ticks);
}

static int usj_wr(const uint8_t *buf, size_t len, TickType_t ticks)
{
    return usb_serial_jtag_write_bytes(buf, len, ticks);
}

static const transport_t T_UART = { "uart0", uart_rd, uart_wr };
static const transport_t T_USJ  = { "usj",   usj_rd,  usj_wr  };

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// Read exactly len bytes before the absolute deadline. Returns true on success.
static bool read_full(const transport_t *t, uint8_t *buf, size_t len, int64_t deadline)
{
    size_t got = 0;
    while (got < len) {
        int64_t left = deadline - now_ms();
        if (left <= 0) return false;
        int n = t->read(buf + got, len - got, pdMS_TO_TICKS(left > 100 ? 100 : left));
        if (n > 0) got += n;
    }
    return true;
}

static void write_full(const transport_t *t, const uint8_t *buf, size_t len)
{
    size_t sent = 0;
    int idle = 0;
    while (sent < len) {
        int n = t->write(buf + sent, len - sent, pdMS_TO_TICKS(100));
        if (n > 0) {
            sent += n;
            idle = 0;
        } else if (++idle > 50) {   // host not reading for 5 s: drop the rest
            return;
        }
    }
}

static void send_reply(const transport_t *t, int16_t status, const uint8_t *body, uint32_t len)
{
    uint8_t hdr[10] = { 'L', 'R', 'S', '1',
                        (uint8_t)(status >> 8), (uint8_t)status,
                        (uint8_t)(len >> 24), (uint8_t)(len >> 16), (uint8_t)(len >> 8), (uint8_t)len };
    write_full(t, hdr, sizeof(hdr));
    if (len) write_full(t, body, len);
}

static void send_text(const transport_t *t, int16_t status, const char *msg)
{
    send_reply(t, status, (const uint8_t *)msg, strlen(msg));
}

// ---------------------------------------------------------------- HTTP client

typedef struct {
    uint8_t *data;
    size_t len;
    size_t cap;
} buf_t;

static bool buf_reserve(buf_t *b, size_t extra)
{
    if (b->len + extra <= b->cap) return true;
    size_t cap = b->cap ? b->cap : 4096;
    while (cap < b->len + extra) cap *= 2;
    uint8_t *p = realloc(b->data, cap);
    if (!p) return false;
    b->data = p;
    b->cap = cap;
    return true;
}

static int connect_with_timeout(char *err, size_t errlen)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        snprintf(err, errlen, "socket() failed: errno %d", errno);
        return -1;
    }
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(CARD_PORT) };
    inet_pton(AF_INET, CARD_IP, &addr.sin_addr);

    int fl = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, fl | O_NONBLOCK);
    int r = connect(s, (struct sockaddr *)&addr, sizeof(addr));
    if (r < 0 && errno != EINPROGRESS) {
        snprintf(err, errlen, "connect failed: errno %d", errno);
        close(s);
        return -1;
    }
    if (r < 0) {
        fd_set wf;
        FD_ZERO(&wf);
        FD_SET(s, &wf);
        struct timeval tv = { .tv_sec = CONNECT_TIMEOUT_MS / 1000, .tv_usec = (CONNECT_TIMEOUT_MS % 1000) * 1000 };
        r = select(s + 1, NULL, &wf, NULL, &tv);
        if (r <= 0) {
            snprintf(err, errlen, "connect timeout (%d ms)", CONNECT_TIMEOUT_MS);
            close(s);
            return -1;
        }
        int so_err = 0;
        socklen_t sl = sizeof(so_err);
        getsockopt(s, SOL_SOCKET, SO_ERROR, &so_err, &sl);
        if (so_err) {
            snprintf(err, errlen, "connect failed: errno %d", so_err);
            close(s);
            return -1;
        }
    }
    fcntl(s, F_SETFL, fl);
    return s;
}

static bool send_all(int s, const void *data, size_t len, int64_t deadline)
{
    const uint8_t *p = data;
    while (len) {
        int64_t left = deadline - now_ms();
        if (left <= 0) return false;
        struct timeval tv = { .tv_sec = left / 1000, .tv_usec = (left % 1000) * 1000 };
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        int n = send(s, p, len, 0);
        if (n <= 0) return false;
        p += n;
        len -= n;
    }
    return true;
}

// Decode a chunked body in place. Returns decoded length, or -1 if malformed.
static long dechunk(uint8_t *p, size_t len)
{
    size_t in = 0, out = 0;
    while (in < len) {
        char *end;
        unsigned long sz = strtoul((char *)p + in, &end, 16);
        uint8_t *eol = memchr(p + in, '\n', len - in);
        if (!eol || (uint8_t *)end == p + in) return -1;
        in = eol - p + 1;
        if (sz == 0) return out;
        if (in + sz > len) sz = len - in;  // truncated last chunk: keep what we have
        memmove(p + out, p + in, sz);
        out += sz;
        in += sz;
        if (in < len && p[in] == '\r') in++;
        if (in < len && p[in] == '\n') in++;
    }
    return out;
}

// Case-insensitive header lookup inside [hdr, hdr+len). Returns pointer to value or NULL.
static const char *find_header(const char *hdr, size_t len, const char *name)
{
    size_t nl = strlen(name);
    const char *p = hdr, *end = hdr + len;
    while (p < end) {
        const char *eol = memchr(p, '\n', end - p);
        if (!eol) eol = end;
        if ((size_t)(eol - p) > nl && strncasecmp(p, name, nl) == 0 && p[nl] == ':') {
            p += nl + 1;
            while (p < eol && (*p == ' ' || *p == '\t')) p++;
            return p;
        }
        p = eol + 1;
    }
    return NULL;
}

// Performs the request and sends the reply on transport t.
static void do_http(const transport_t *t, char method, const char *path, const uint8_t *body, uint32_t body_len)
{
    char err[96];
    int64_t deadline = now_ms() + HTTP_TIMEOUT_MS;

    int s = connect_with_timeout(err, sizeof(err));
    if (s < 0) {
        send_text(t, ST_HTTP_FAIL, err);
        return;
    }

    char hdr[MAX_PATH + 256];
    int hl;
    if (method == 'P') {
        hl = snprintf(hdr, sizeof(hdr),
                      "POST %s HTTP/1.1\r\nHost: " CARD_IP "\r\n"
                      "Content-Type: application/json;charset=UTF-8\r\nAccept: text/json\r\n"
                      "Content-Length: %lu\r\nConnection: close\r\n\r\n",
                      path, (unsigned long)body_len);
    } else {
        hl = snprintf(hdr, sizeof(hdr),
                      "GET %s HTTP/1.1\r\nHost: " CARD_IP "\r\nConnection: close\r\n\r\n", path);
    }

    if (!send_all(s, hdr, hl, deadline) || (method == 'P' && body_len && !send_all(s, body, body_len, deadline))) {
        close(s);
        send_text(t, ST_HTTP_FAIL, "send failed / timeout");
        return;
    }

    // Read until the card closes the connection.
    buf_t rx = { 0 };
    bool timed_out = false, oom = false;
    for (;;) {
        int64_t left = deadline - now_ms();
        if (left <= 0) { timed_out = true; break; }
        struct timeval tv = { .tv_sec = left / 1000, .tv_usec = (left % 1000) * 1000 };
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        if (!buf_reserve(&rx, 2048)) { oom = true; break; }
        int n = recv(s, rx.data + rx.len, rx.cap - rx.len, 0);
        if (n > 0) {
            rx.len += n;
        } else if (n == 0) {
            break;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) timed_out = true;
            else if (rx.len == 0) snprintf(err, sizeof(err), "recv failed: errno %d", errno);
            break;
        }
    }
    close(s);

    if (oom) {
        free(rx.data);
        send_text(t, ST_NO_MEM, "out of memory reading response");
        return;
    }

    // Locate end of headers.
    uint8_t *sep = NULL;
    for (size_t i = 0; i + 3 < rx.len; i++) {
        if (memcmp(rx.data + i, "\r\n\r\n", 4) == 0) { sep = rx.data + i; break; }
    }
    if (!sep) {
        free(rx.data);
        if (timed_out) send_text(t, ST_HTTP_FAIL, "http timeout");
        else if (rx.len == 0) send_text(t, ST_HTTP_FAIL, "empty response");
        else send_text(t, ST_HTTP_FAIL, "bad http response");
        return;
    }

    size_t hdr_len = sep - rx.data;
    int code = 0;
    if (sscanf((char *)rx.data, "HTTP/%*d.%*d %d", &code) != 1) {
        free(rx.data);
        send_text(t, ST_HTTP_FAIL, "bad status line");
        return;
    }

    uint8_t *bp = sep + 4;
    size_t blen = rx.len - (bp - rx.data);

    // Null-terminate header block for parsing (overwrites the first '\r' of the separator).
    *sep = 0;
    const char *te = find_header((char *)rx.data, hdr_len, "Transfer-Encoding");
    const char *cl = find_header((char *)rx.data, hdr_len, "Content-Length");
    if (te && strncasecmp(te, "chunked", 7) == 0) {
        long d = dechunk(bp, blen);
        if (d >= 0) blen = d;
    } else if (cl) {
        unsigned long want = strtoul(cl, NULL, 10);
        if (want < blen) blen = want;
    }

    send_reply(t, (int16_t)code, bp, blen);
    free(rx.data);
}

// ---------------------------------------------------------------- request handling

static void do_info(const transport_t *t)
{
    wifi_ap_record_t ap;
    int rssi = 0;
    if (s_wifi_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    char js[160];
    int n = snprintf(js, sizeof(js),
                     "{\"wifi\":%s,\"ip\":\"%s\",\"rssi\":%d,\"heap\":%lu,\"up_ms\":%lld}",
                     s_wifi_up ? "true" : "false", s_wifi_up ? s_ip : "", rssi,
                     (unsigned long)esp_get_free_heap_size(), (long long)now_ms());
    send_reply(t, 200, (uint8_t *)js, n);
}

static void serve(void *arg)
{
    const transport_t *t = arg;
    uint8_t win[4] = { 0 };

    for (;;) {
        // Scan byte-by-byte for the magic.
        uint8_t c;
        if (t->read(&c, 1, portMAX_DELAY) != 1) continue;
        win[0] = win[1]; win[1] = win[2]; win[2] = win[3]; win[3] = c;
        if (memcmp(win, "LRQ1", 4) != 0) continue;
        memset(win, 0, sizeof(win));

        int64_t deadline = now_ms() + FRAME_TIMEOUT_MS;
        uint8_t h[7];
        if (!read_full(t, h, sizeof(h), deadline)) continue;
        char method = h[0];
        uint16_t path_len = (h[1] << 8) | h[2];
        uint32_t body_len = ((uint32_t)h[3] << 24) | ((uint32_t)h[4] << 16) | ((uint32_t)h[5] << 8) | h[6];

        if (path_len > MAX_PATH || body_len > MAX_BODY) {
            xSemaphoreTake(s_req_lock, portMAX_DELAY);
            send_text(t, ST_BAD_REQ, "path or body too long");
            xSemaphoreGive(s_req_lock);
            continue;
        }

        char path[MAX_PATH + 1];
        if (!read_full(t, (uint8_t *)path, path_len, deadline)) continue;
        path[path_len] = 0;

        uint8_t *body = NULL;
        bool oom = false;
        if (body_len) {
            body = malloc(body_len);
            if (!body) {
                // Still drain the body so the stream stays in sync.
                oom = true;
                uint8_t sink[256];
                uint32_t left = body_len;
                bool ok = true;
                while (left && ok) {
                    uint32_t n = left > sizeof(sink) ? sizeof(sink) : left;
                    ok = read_full(t, sink, n, deadline);
                    left -= n;
                }
                if (!ok) continue;
            } else if (!read_full(t, body, body_len, deadline)) {
                free(body);
                continue;
            }
        }

        xSemaphoreTake(s_req_lock, portMAX_DELAY);
        if (oom) {
            send_text(t, ST_NO_MEM, "out of memory for request body");
        } else if (method == 'I') {
            do_info(t);
        } else if (method != 'P' && method != 'G') {
            send_text(t, ST_BAD_REQ, "unknown method");
        } else if (!s_wifi_up) {
            send_text(t, ST_NO_WIFI, "wifi not connected to " WIFI_SSID);
        } else {
            do_http(t, method, path_len ? path : "/", body, body_len);
        }
        xSemaphoreGive(s_req_lock);
        free(body);
    }
}

// ---------------------------------------------------------------- WiFi

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_up = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_wifi_up = true;
    }
}

static void wifi_init(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);

    wifi_config_t wc = { 0 };
    strcpy((char *)wc.sta.ssid, WIFI_SSID);
    strcpy((char *)wc.sta.password, WIFI_PASS);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

// ---------------------------------------------------------------- main

void app_main(void)
{
    s_req_lock = xSemaphoreCreateMutex();

    usb_serial_jtag_driver_config_t usj = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = TX_BUF_SIZE,
    };
    usb_serial_jtag_driver_install(&usj);

    uart_config_t uc = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uc);
    uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    wifi_init();

    xTaskCreate(serve, "serve_usj", 6144, (void *)&T_USJ, 5, NULL);
    xTaskCreate(serve, "serve_uart", 6144, (void *)&T_UART, 5, NULL);
}
