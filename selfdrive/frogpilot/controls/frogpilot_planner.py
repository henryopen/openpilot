import numpy as np

import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import StreamingMovingAverage

from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, COMFORT_BRAKE, DANGER_ZONE_COST, J_EGO_COST, STOP_DISTANCE, \
                                                                           get_jerk_factor, get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, Lead, get_max_accel
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_functions import calculate_lane_width, calculate_road_curvature
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED, TRAJECTORY_SIZE
from openpilot.selfdrive.frogpilot.controls.lib.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

A_CRUISE_MIN_ECO = A_CRUISE_MIN
A_CRUISE_MIN_SPORT = A_CRUISE_MIN / 2
                  # MPH = [ 0.,  11,  22,  34,  45,  56,  89]
                  # KPH = [ 0.,  18,  36,  54,  72,  90,  144]
A_CRUISE_MAX_BP_CUSTOM =  [ 0.,  5., 10., 15., 20., 25., 40.]
A_CRUISE_MAX_VALS_ECO =   [2.5, 1.5, 1.0, 0.6, 0.5, 0.4, 0.3]
A_CRUISE_MAX_VALS_SPORT = [4.0, 3.0, 2.0, 1.0, 0.9, 0.8, 0.6]

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]

TARGET_LAT_A = 1.9  # m/s^2

def get_max_accel_eco(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO)

def get_max_accel_sport(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT)

class FrogPilotPlanner:
  def __init__(self):
    self.params_memory = Params("/dev/shm/params")

    self.cem = ConditionalExperimentalMode()
    self.lead_one = Lead()
    self.mtsc = MapTurnSpeedController()

    self.lead_departing = False
    self.override_force_stop = False
    self.override_slc = False
    self.slower_lead = False
    self.taking_curve_quickly = False
    self.tracking_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.model_length = 0
    self.mtsc_target = 0
    self.overridden_speed = 0
    self.road_curvature = 0
    self.slc_target = 0
    self.speed_jerk = 0
    self.tracked_model_length = 0
    self.tracking_lead_distance = 0
    self.v_cruise = 0
    self.vtsc_target = 0
    self.detect_speed_prev = 0
    self.trafficState = 0
    self.trafficState1 = 0
    self.changelane_ct = 0
    self.changelane = False
    self.stopdrel = 5.0

  def update(self, carState, controlsState, frogpilotCarControl, frogpilotCarState, frogpilotNavigation, modelData, radarState, frogpilot_toggles):
    if frogpilot_toggles.radarless_model:
      model_leads = list(modelData.leadsV3)
      if len(model_leads) > 0:
        model_lead = model_leads[0]
        self.lead_one.update(model_lead.x[0], model_lead.y[0], model_lead.v[0], model_lead.a[0], model_lead.prob)
      else:
        self.lead_one.reset()
    else:
      self.lead_one = radarState.leadOne

    v_cruise = min(controlsState.vCruise, 255) * CV.KPH_TO_MS
    v_ego = max(carState.vEgo, 0)
    v_ego_kph = v_ego *3.6
    v_lead = self.lead_one.vLead

    distance_offset = max(frogpilot_toggles.increased_stopping_distance + min(CITY_SPEED_LIMIT - v_ego, 0), 0) if not frogpilotCarState.trafficModeActive else 0
    lead_distance = self.lead_one.dRel - distance_offset
    dvratio = self.lead_one.dRel/np.where(v_ego_kph < 1, 1, v_ego_kph)
    stopping_distance = STOP_DISTANCE + distance_offset
    self.params_memory.put_int("ADrel",self.lead_one.dRel)
    self.params_memory.put_int("AdvRatio",(self.lead_one.dRel/np.where(v_ego < 1, 1, v_ego))*100)

    if frogpilot_toggles.conditional_experimental_mode and controlsState.enabled:
      self.cem.update(carState, frogpilotNavigation, self.lead_one, modelData, self.model_length, self.road_curvature, self.slower_lead, self.tracking_lead, self.v_cruise, v_ego, v_lead, frogpilot_toggles, dvratio, v_ego_kph, self.lead_one.status)

    check_lane_width = frogpilot_toggles.adjacent_lanes or frogpilot_toggles.blind_spot_path or frogpilot_toggles.lane_detection
    if check_lane_width and v_ego >= frogpilot_toggles.minimum_lane_change_speed:
      self.lane_width_left = float(calculate_lane_width(modelData.laneLines[0], modelData.laneLines[1], modelData.roadEdges[0]))
      self.lane_width_right = float(calculate_lane_width(modelData.laneLines[3], modelData.laneLines[2], modelData.roadEdges[1]))
    else:
      self.lane_width_left = 0
      self.lane_width_right = 0

    if self.params_memory.get_bool("AutoTurn"):
      if v_cruise-v_ego > 4 and v_ego > 20 and self.lead_one.status and (0.4 < dvratio < 0.8) and not self.params_memory.get_bool("AutoChange"):
        self.changelane_ct += 1
        if self.changelane_ct > 60:
          if self.params_memory.get_int("KeyTurnLight") == 0:
            if self.lane_width_left > 3.0 and not carState.leftBlindspot:
              self.params_memory.put_int("KeyTurnLight", 1)
            elif self.lane_width_right > 3.0 and not carState.rightBlindspot:
              self.params_memory.put_int("KeyTurnLight", 2)
            self.params_memory.put_bool("AutoChange", True)
            self.changelane_ct = 0

    if frogpilot_toggles.lead_departing_alert and self.tracking_lead and carState.standstill and controlsState.enabled:
      self.lead_departing = self.lead_one.dRel - self.tracking_lead_distance > 1.0
      self.lead_departing &= v_lead > 1.0
    else:
      self.lead_departing = False

    self.model_length = modelData.position.x[TRAJECTORY_SIZE - 1]
    self.trafficState1 = int(self.model_length*10)
    self.params_memory.put_int("TrafficState1",self.trafficState1)

    if self.model_length < 10.0 and carState.standstill and self.trafficState == 0:
      self.trafficState = 1
      self.stopdrel = max(self.lead_one.dRel,2.0)
    if self.trafficState == 1:
      if len(modelData.position.x) == TRAJECTORY_SIZE and len(modelData.orientation.x) == TRAJECTORY_SIZE:
        if self.model_length > 39.0:
          self.trafficState = 2
      # if self.lead_one.status and self.lead_one.dRel > 6.0 and self.lead_one.dRel < 12.0:
      #   self.trafficState = 3
    if self.trafficState == 2:
      if v_ego_kph > 8.0:
        self.trafficState = 0
    if not (controlsState.enabled and frogpilotCarState.ecoGear):
      self.trafficState = 0
    self.params_memory.put_int("TrafficState",self.trafficState)

    if self.params_memory.get_bool("AutoAcce"):
        outputaccel_prev = self.params_memory.get_int("AutoAcce")
        # if self.trafficState == 3:
        #   if self.lead_one.status and self.lead_one.dRel > 3.6:
        #     outputaccel = 50
        #   else:
        #     outputaccel = 0
        if self.trafficState == 2:
          if self.lead_one.status:
            if self.lead_one.dRel > self.stopdrel+0.6 and self.lead_one.dRel < self.stopdrel+7.0 and self.lead_one.dRel > 2.6:
              outputaccel = 70
            else:
              outputaccel = 0
          else:
            outputaccel = 70
        else:
          outputaccel = 0

        if outputaccel != outputaccel_prev:
          self.params_memory.put_int("KeyAcce",outputaccel)

    self.road_curvature = abs(float(calculate_road_curvature(modelData, v_ego)))

    if frogpilot_toggles.random_events:
      self.taking_curve_quickly = v_ego > (1 / self.road_curvature)**0.5 * 2 > CRUISING_SPEED * 2 and abs(carState.steeringAngleDeg) > 30

    if v_ego > CRUISING_SPEED:
      self.override_force_stop = False
      self.tracking_lead = self.lead_one.status
      self.tracked_model_length = 0
    elif not carState.standstill and self.tracking_lead and self.lead_one.dRel < CITY_SPEED_LIMIT:
      self.tracking_lead_distance = self.lead_one.dRel
    elif carState.standstill and frogpilot_toggles.force_stops:
      self.override_force_stop = True
    else:
      self.tracking_lead &= self.lead_one.status

    self.set_acceleration(controlsState, frogpilotCarState, v_cruise, v_ego, frogpilot_toggles)
    self.set_follow_values(controlsState, frogpilotCarState, v_ego, v_lead, frogpilot_toggles)
    self.update_follow_values(lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles)
    self.update_v_cruise(carState, controlsState, frogpilotCarState, frogpilotNavigation, modelData, v_cruise, v_ego, frogpilot_toggles)

  def _update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median=True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x

  def set_acceleration(self, controlsState, frogpilotCarState, v_cruise, v_ego, frogpilot_toggles):
    eco_gear = frogpilotCarState.ecoGear
    sport_gear = frogpilotCarState.sportGear

    if self.lead_one.status and frogpilot_toggles.aggressive_acceleration:
      self.max_accel = float(np.clip(self.lead_one.aLeadK, get_max_accel_sport(v_ego), 2.0 if v_ego >= 20 else 4.0))
    elif frogpilot_toggles.map_acceleration and (eco_gear or sport_gear):
      if eco_gear:
        self.max_accel = get_max_accel_eco(v_ego)
      else:
        self.max_accel = get_max_accel_sport(v_ego)
    else:
      if frogpilot_toggles.acceleration_profile == 1:
        self.max_accel = get_max_accel_eco(v_ego)
      elif frogpilot_toggles.acceleration_profile in (2, 3):
        self.max_accel = get_max_accel_sport(v_ego)
      elif controlsState.experimentalMode:
        self.max_accel = ACCEL_MAX
      else:
        self.max_accel = get_max_accel(v_ego)

    if not self.tracking_lead:
      self.max_accel = float(min(self.max_accel, self.max_accel * (self.v_cruise / CITY_SPEED_LIMIT)))

    if controlsState.experimentalMode:
      self.min_accel = ACCEL_MIN
    elif min(self.mtsc_target, self.vtsc_target) < v_cruise:
      self.min_accel = A_CRUISE_MIN
    elif frogpilot_toggles.map_deceleration and (eco_gear or sport_gear):
      if eco_gear:
        self.min_accel = A_CRUISE_MIN_ECO
      else:
        self.min_accel = A_CRUISE_MIN_SPORT
    else:
      if frogpilot_toggles.deceleration_profile == 1:
        self.min_accel = A_CRUISE_MIN_ECO
      elif frogpilot_toggles.deceleration_profile == 2:
        self.min_accel = A_CRUISE_MIN_SPORT
      else:
        self.min_accel = A_CRUISE_MIN

  def set_follow_values(self, controlsState, frogpilotCarState, v_ego, v_lead, frogpilot_toggles):
    if frogpilotCarState.trafficModeActive:
      self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration)
      self.base_danger_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger)
      self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed)
      self.t_follow = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_t_follow)
    else:
      self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
        frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
        frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
        frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
        frogpilot_toggles.custom_personalities, controlsState.personality
      )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.custom_personalities, frogpilot_toggles.aggressive_follow, frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow, controlsState.personality
      )

    if self.tracking_lead:
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
    else:
      self.acceleration_jerk = self.base_acceleration_jerk
      self.danger_jerk = self.base_danger_jerk
      self.speed_jerk = self.base_speed_jerk
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0

  def update_follow_values(self, lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles):
    # Offset by FrogAi for FrogPilot for a more natural approach to a faster lead
    if frogpilot_toggles.aggressive_acceleration and v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      standstill_offset = max(stopping_distance - v_ego, 0) * max(v_lead - v_ego, 0)
      acceleration_offset = np.clip((v_lead - v_ego) + standstill_offset - COMFORT_BRAKE, 1, distance_factor)
      self.acceleration_jerk = self.base_acceleration_jerk / acceleration_offset
      self.danger_jerk = self.base_danger_jerk / acceleration_offset
      self.speed_jerk = self.base_speed_jerk / acceleration_offset
      self.t_follow /= acceleration_offset

    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if (frogpilot_toggles.conditional_experimental_mode or frogpilot_toggles.smoother_braking) and v_lead < v_ego:
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      far_lead_offset = max(lead_distance - (v_ego * self.t_follow) - stopping_distance + (v_lead - CITY_SPEED_LIMIT), 0)
      braking_offset = np.clip((v_ego - v_lead) + far_lead_offset - COMFORT_BRAKE, 1, distance_factor)
      if frogpilot_toggles.smoother_braking:
        self.acceleration_jerk = self.base_acceleration_jerk * min(braking_offset, COMFORT_BRAKE / 2)
        self.danger_jerk = self.base_danger_jerk * min(braking_offset, COMFORT_BRAKE / 2)
        self.speed_jerk = self.base_speed_jerk * min(braking_offset, COMFORT_BRAKE * 2)
        self.t_follow /= braking_offset
      self.slower_lead = max(braking_offset - far_lead_offset, 1) > 1

  def update_v_cruise(self, carState, controlsState, frogpilotCarState, frogpilotNavigation, modelData, v_cruise, v_ego, frogpilot_toggles):

    # Pfeiferj's Map Turn Speed Controller
    if frogpilot_toggles.map_turn_speed_controller and v_ego > CRUISING_SPEED and controlsState.enabled:
      mtsc_active = self.mtsc_target < v_cruise
      self.mtsc_target = np.clip(self.mtsc.target_speed(v_ego, carState.aEgo), CRUISING_SPEED, v_cruise)

      if frogpilot_toggles.mtsc_curvature_check and self.road_curvature < 1.0 and not mtsc_active:
        self.mtsc_target = v_cruise
      if self.mtsc_target == CRUISING_SPEED:
        self.mtsc_target = v_cruise
    else:
      self.mtsc_target = v_cruise if v_cruise != 255 else 0

    # Pfeiferj's Speed Limit Controller
    SpeedLimitController.update(frogpilotCarState.dashboardSpeedLimit, frogpilotNavigation.navigationSpeedLimit, v_ego, frogpilot_toggles)
    self.slc_target = SpeedLimitController.desired_speed_limit
    detect_sl = self.slc_target * 3.6
    if self.params_memory.get_bool("SLC"):
      if detect_sl != self.detect_speed_prev and v_ego*3.6 > 5.0:
        if detect_sl > 0:
          self.params_memory.put_int('DetectSpeedLimit', detect_sl)
          self.params_memory.put_bool('SpeedLimitChanged', True)
          self.detect_speed_prev = detect_sl
        else:
          self.detect_speed_prev = 0
      else:
        self.params_memory.put_bool('SpeedLimitChanged', False)
    else:
      self.params_memory.put_bool('SpeedLimitChanged', False)
      self.params_memory.put_int('DetectSpeedLimit', detect_sl)

    # Pfeiferj's Vision Turn Controller
    if frogpilot_toggles.vision_turn_controller and v_ego > CRUISING_SPEED and controlsState.enabled:
      adjusted_road_curvature = self.road_curvature * frogpilot_toggles.curve_sensitivity
      adjusted_target_lat_a = TARGET_LAT_A * frogpilot_toggles.turn_aggressiveness

      self.vtsc_target = (adjusted_target_lat_a / adjusted_road_curvature)**0.5
      self.vtsc_target = np.clip(self.vtsc_target, CRUISING_SPEED, v_cruise)
    else:
      self.vtsc_target = v_cruise if v_cruise != 255 else 0

    if (frogpilot_toggles.force_standstill or frogpilot_toggles.force_stops) and v_ego < 1 and not self.override_force_stop:
      if carState.gasPressed:
        self.override_force_stop = True
      else:
        self.v_cruise = -1

    elif frogpilot_toggles.force_stops and v_ego < CRUISING_SPEED * 2 and controlsState.experimentalMode and not self.override_force_stop:
      if carState.gasPressed or self.tracking_lead or abs(carState.steeringAngleDeg) > 15:
        self.override_force_stop = True
      else:
        if self.tracked_model_length == 0:
          self.tracked_model_length = self.model_length

        self.tracked_model_length -= v_ego * DT_MDL
        self.v_cruise = self.tracked_model_length / ModelConstants.T_IDXS[TRAJECTORY_SIZE - 1]

    else:
      targets = [self.mtsc_target, self.vtsc_target]
      self.v_cruise = min([target if target > CRUISING_SPEED else v_cruise for target in targets])

  def publish(self, sm, pm, frogpilot_toggles):
    frogpilot_plan_send = messaging.new_message('frogpilotPlan')
    frogpilot_plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    frogpilotPlan.accelerationJerk = A_CHANGE_COST * float(self.acceleration_jerk)
    frogpilotPlan.accelerationJerkStock = A_CHANGE_COST * float(self.base_acceleration_jerk)
    frogpilotPlan.dangerJerk = DANGER_ZONE_COST * float(self.danger_jerk)
    frogpilotPlan.speedJerk = J_EGO_COST * float(self.speed_jerk)
    frogpilotPlan.speedJerkStock = J_EGO_COST * float(self.base_speed_jerk)
    frogpilotPlan.tFollow = float(self.t_follow)

    frogpilotPlan.adjustedCruise = float(min(self.mtsc_target, self.vtsc_target) * (CV.MS_TO_KPH if frogpilot_toggles.is_metric else CV.MS_TO_MPH))
    frogpilotPlan.vtscControllingCurve = bool(self.mtsc_target > self.vtsc_target)

    frogpilotPlan.conditionalExperimentalActive = bool(self.cem.experimental_mode)

    frogpilotPlan.desiredFollowDistance = self.safe_obstacle_distance - self.stopped_equivalence_factor
    frogpilotPlan.safeObstacleDistance = self.safe_obstacle_distance
    frogpilotPlan.safeObstacleDistanceStock = self.safe_obstacle_distance_stock
    frogpilotPlan.stoppedEquivalenceFactor = self.stopped_equivalence_factor

    #frogpilotPlan.greenLight = self.model_length > TRAJECTORY_SIZE

    frogpilotPlan.greenLight = (self.trafficState == 2) or (self.trafficState == 4)

    frogpilotPlan.laneWidthLeft = self.lane_width_left
    frogpilotPlan.laneWidthRight = self.lane_width_right

    frogpilotPlan.leadDeparting = self.lead_departing

    frogpilotPlan.maxAcceleration = self.max_accel
    frogpilotPlan.minAcceleration = self.min_accel

    frogpilotPlan.roadCurvature = self.road_curvature

    frogpilotPlan.slcOverridden = bool(self.override_slc)
    frogpilotPlan.slcOverriddenSpeed = float(self.overridden_speed)
    frogpilotPlan.slcSpeedLimit = self.slc_target
    frogpilotPlan.slcSpeedLimitOffset = SpeedLimitController.offset
    frogpilotPlan.unconfirmedSlcSpeedLimit = SpeedLimitController.desired_speed_limit

    frogpilotPlan.takingCurveQuickly = bool(self.taking_curve_quickly)

    frogpilotPlan.vCruise = float(self.v_cruise)

    pm.send('frogpilotPlan', frogpilot_plan_send)
