#!/usr/bin/env python3
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.selfdrive.monitoring.policy import AlertLevel, DriverMonitoring


def neutralize_dm(DM: DriverMonitoring) -> None:
  # DM off: wind the policy back to fully attentive after every step, rather than
  # editing the packet it hands out. Nothing accumulates, so there is no lockout, no
  # alert escalation, no DriverLockoutCount write, and the driver view keeps reading
  # 100% awareness. Face and pose state is deliberately untouched so the cabin camera
  # dialog still draws the face box and the pose calibration keeps filling in.
  DM._reset_awareness()
  DM.alert_level = AlertLevel.none
  DM.lockout_active = False
  DM.lockout_count = 0
  DM.lockout_time_elapsed = 0
  DM.alert_3_cnt = 0
  DM.cnt_since_alert_3 = 0
  DM.no_response_cnt = 0
  DM.dcam_uncertain_cnt = 0


def dmonitoringd_thread():
  config_realtime_process([0, 1, 2, 3], 5)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  sm = messaging.SubMaster(['driverStateV2', 'extrinsicsCalibration', 'carState', 'selfdriveState', 'modelV2'], poll='driverStateV2')

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  demo_mode=False
  disable_dm = params.get_bool("DisableDriverMonitoring")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      # iterate when model has new output
      continue

    valid = sm.all_checks()
    if demo_mode and sm.valid['driverStateV2']:
      DM.run_step(sm, demo=True)
    elif valid:
      DM.run_step(sm, demo=demo_mode)

    if disable_dm:
      neutralize_dm(DM)

    # publish
    dat = DM.get_state_packet(valid=valid)
    pm.send('driverMonitoringState', dat)

    # load live always-on toggle
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")
      disable_dm = params.get_bool("DisableDriverMonitoring")

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and not demo_mode and
     DM.wheelpos_offsetter.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_offsetter.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
