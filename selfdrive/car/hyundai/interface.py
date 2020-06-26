#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase, MAX_CTRL_SPEED
from selfdrive.car.hyundai.carstate import ATOMC

GearShifter = car.CarState.GearShifter

#from selfdrive.kegman_conf import kegman_conf

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.cp2 = self.CS.get_can2_parser(CP)
    self.lkas_button_alert = False


    self.meg_timer = 0
    self.meg_name = 0

  """
    self.steer_Kf1 = [0.00001,0.000015]    
    self.steer_Ki1 = [0.01,0.01]
    self.steer_Kp1 = [0.11,0.12]
    self.steer_Kf2 = [0.00005,0.00005]
    self.steer_Ki2 = [0.04,0.05]
    self.steer_Kp2 = [0.20,0.25]
    self.deadzone = 0.0
    self.steerAngleOffset = 1
    self.load_tune( CP )
  #@staticmethod
  def load_tune(self, CP):
    # live tuning through /data/openpilot/tune.py overrides interface.py settings
    self.kegman = kegman_conf(CP)
    self.steer_Kp1 = [ float(self.kegman.conf['Kp']), float(self.kegman.conf['sR_Kp']) ]
    self.steer_Ki1 = [ float(self.kegman.conf['Ki']), float(self.kegman.conf['sR_Ki']) ]
    self.steer_Kf1 = [ float(self.kegman.conf['Kf']), float(self.kegman.conf['sR_Kf']) ]
    self.steer_Kp2 = [ float(self.kegman.conf['Kp2']), float(self.kegman.conf['sR_Kp2']) ]
    self.steer_Ki2 = [ float(self.kegman.conf['Ki2']), float(self.kegman.conf['sR_Ki2']) ]
    self.steer_Kf2 = [ float(self.kegman.conf['Kf2']), float(self.kegman.conf['sR_Kf2']) ]        
    self.deadzone = float(self.kegman.conf['deadzone'])
    try:
      self.steerAngleOffset = float(self.kegman.conf['steerAngleOffset'])
    except:
      self.steerAngleOffset = 0
    finally:  # try end 
      pass   
  """

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    global ATOMC    
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai
    ret.radarOffCan = False  #False(선행차우선)  #True(차선우선)    #선행차량 인식 마크 유무.

    # Hyundai port is a community feature for now
    ret.communityFeature = False  #True

    ret.longcontrolEnabled = False

    """
      0.7.5
      ret.steerActuatorDelay = 0.1  # Default delay   0.1
      ret.steerRateCost = 0.5
      ret.steerLimitTimer = 0.4
      tire_stiffness_factor = 1
    """

    """
      0.7.3
      ret.steerActuatorDelay = 0.10  # Default delay   0.15
      ret.steerRateCost = 0.45
      ret.steerLimitTimer = 0.8
      tire_stiffness_factor = 0.7
    """

    tire_stiffness_factor = 1.
    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]

    if candidate == CAR.GRANDEUR_HYBRID:
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.845

      # 1번 튜닝.
      #ret.steerRatio = 12.37  #12.5
      #ret.steerRateCost = 0.5 #0.4
      #ret.lateralTuning.pid.kf = 0.00003 
      #ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.20], [0.03, 0.04]]



      # 2번 튜닝.
      ret.steerRatio = ATOMC.steerRatio  #10.5  #12.5
      ret.steerRateCost = ATOMC.steerRateCost #0.4 #0.4
      ret.lateralTuning.pid.kf = ATOMC.steer_Kp1[0] #0.00001
      ret.lateralTuning.pid.kpV = ATOMC.steer_Kp1  #[0.12, 0.15]
      ret.lateralTuning.pid.kiV = ATOMC.steer_Ki1  #[0.02, 0.02]



      #ret.lateralTuning_sR.kf = 0.00005
      #ret.lateralTuning_sR.sRBP = [4, 30.]
      #ret.lateralTuning_sR.kpV = [0.12, 0.15]
      #ret.lateralTuning_sR.kiV = [0.02, 0.02]


      # indi
      #ret.lateralTuning.init('indi')
      #ret.lateralTuning.indi.innerLoopGain = 3.0
      #ret.lateralTuning.indi.outerLoopGain = 2.0
      #ret.lateralTuning.indi.timeConstant = 1.0
      #ret.lateralTuning.indi.actuatorEffectiveness = 1.5
      #ret.steerRatio = 9.0 


      #ret.steerActuatorDelay = 0.1 # Stinger GT Limited AWD 3.3T stock value (Tunder's 2020) 
      #ret.steerLimitTimer = 0.4 # stock is 0.01 but 0.04 seems to work well
      #tire_stiffness_factor = 1.125 # LiveParameters (Tunder's 2020)

 
    elif candidate == CAR.SANTAFE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2, 0.35], [0.05, 0.09]]
    elif candidate == CAR.SONATA:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.SONATA_2019:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.PALISADE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 13.75 * 1.15
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KIA_SORENTO:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
      ret.minSteerSpeed = 60 * CV.KPH_TO_MS
    elif candidate == CAR.GENESIS_G80:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
    elif candidate in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_H]:
      #ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      #ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      #ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
      ret.steerRatio = ATOMC.steerRatio  #10.5  #12.5
      ret.steerRateCost = ATOMC.steerRateCost #0.4 #0.4
      ret.lateralTuning.pid.kf = ATOMC.steer_Kp1[0] #0.00001
      ret.lateralTuning.pid.kpV = ATOMC.steer_Kp1  #[0.12, 0.15]
      ret.lateralTuning.pid.kiV = ATOMC.steer_Ki1  #[0.02, 0.02]
    elif candidate == CAR.KIA_STINGER:
      #ret.lateralTuning.pid.kf = 0.00005
      #ret.mass = 1825. + STD_CARGO_KG
      #ret.wheelbase = 2.78
      #ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
      #ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]

      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGain = 3.0
      ret.lateralTuning.indi.outerLoopGain = 2.0
      ret.lateralTuning.indi.timeConstant = 1.0
      ret.lateralTuning.indi.actuatorEffectiveness = 1.5
      ret.steerActuatorDelay = 0.08 # Stinger GT Limited AWD 3.3T stock value (Tunder's 2020) 
      ret.steerLimitTimer = 0.4 # stock is 0.01 but 0.04 seems to work well
      tire_stiffness_factor = 1.125 # LiveParameters (Tunder's 2020)
      ret.steerRateCost = 1.0
      ret.mass = 1825.0 + STD_CARGO_KG
      ret.wheelbase = 2.906 # https://www.kia.com/us/en/stinger/specs
      ret.steerRatio = 13.56   # 10.28 measured by wheel alignment machine/reported steering angle by OP. 2020 GT Limited AWD has a variable steering ratio ultimately ending in 10.28.  13.56 after 1200km in LiveParamaters (Tunder)
    elif candidate == CAR.KONA:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KONA_EV:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KIA_FORTE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)


    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    ret.longitudinalTuning.kpBP = [0., 10., 40.]
    ret.longitudinalTuning.kpV = [1.2, 0.6, 0.2]
    ret.longitudinalTuning.kiBP = [0., 10., 30., 40.]
    ret.longitudinalTuning.kiV = [0.05, 0.02, 0.01, 0.005]
    ret.longitudinalTuning.deadzoneBP = [0., 40]
    ret.longitudinalTuning.deadzoneV = [0., 0.02]


    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [1.0]
    ret.gasMaxBP = [0., 10., 40.]
    ret.gasMaxV = [0.5, 0.5, 0.5]
    ret.brakeMaxBP = [0., 20.]
    ret.brakeMaxV = [1., 0.8]

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay

    ret.stoppingControl = True
    ret.startAccel = 0.0

    # ignore CAN2 address if L-CAN on the same BUS
    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1
    ret.radarOffCan = ret.sccBus == -1
    ret.openpilotLongitudinalControl = False #TODO make ui toggle
    ret.enableCruise = not ret.radarOffCan
    ret.autoLcaEnabled = False
    ret.spasEnabled = False

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid

    if self.CP.enableCruise and not self.CC.scc_live:
      self.CP.enableCruise = False
    elif self.CC.scc_live and not self.CP.enableCruise:
      self.CP.enableCruise = True

    # most HKG cars has no long control, it is safer and easier to engage by main on
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.enabled = ret.cruiseState.available
    # some Optima only has blinker flash signal
    #if self.CP.carFingerprint == CAR.KIA_OPTIMA:
    #  ret.leftBlinker = bool(self.CS.left_blinker_flash or self.CS.prev_left_blinker and self.CC.turning_signal_timer)
    #  ret.rightBlinker = bool(self.CS.right_blinker_flash or self.CS.prev_right_blinker and self.CC.turning_signal_timer)

    # turning indicator alert logic
    if (ret.leftBlinker or ret.rightBlinker or self.CC.turning_signal_timer) and ret.vEgo < LANE_CHANGE_SPEED_MIN - 1.2:
      self.CC.turning_indicator_alert = True 
    else:
      self.CC.turning_indicator_alert = False

    # LKAS button alert logic: reverse on/off
    #if not self.CS.lkas_error and self.CS.lkas_button_on != self.CS.prev_lkas_button_on:
      #self.CC.lkas_button_on = not self.CC.lkas_button_on
      #self.lkas_button_alert = not self.CC.lkas_button_on

    # TODO: button presses
    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      buttonEvents.append(be)
    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents
    #ret.buttonEvents = []    

    events = self.create_common_events(ret)


    
    if not self.cruise_enabled_prev:
      self.meg_timer = 0
      self.meg_name =  None
    else:
      meg_timer = 100
      if self.meg_timer:
        self.meg_timer -= 1
        meg_timer = 0
      #elif not self.CS.lkas_button_on:
      #  self.meg_name = EventName.invalidLkasSetting
      elif ret.cruiseState.standstill:
        self.meg_name = EventName.resumeRequired       
      elif self.CC.lane_change_torque_lower:
        self.meg_name = EventName.laneChangeManual
      elif self.CC.steer_torque_over_timer and self.CC.steer_torque_ratio < 0.1:
        self.meg_name = EventName.steerTorqueOver
      elif self.CC.steer_torque_ratio < 0.5 and self.CS.clu_Vanz > 5:
        self.meg_name = EventName.steerTorqueLow
      elif ret.vEgo > MAX_CTRL_SPEED:
        self.meg_name = EventName.speedTooHigh
      elif ret.steerError:
        self.meg_name = EventName.steerUnavailable
      elif ret.steerWarning:
        self.meg_name = EventName.steerTempUnavailable
      else:
        meg_timer = 0
        self.meg_name =  None

      if meg_timer != 0:
        self.meg_timer = 100

      if self.meg_timer and  self.meg_name != None:
        events.add( self.meg_name )
    

    #TODO: addd abs(self.CS.angle_steers) > 90 to 'steerTempUnavailable' event

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c, sm ):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, sm, c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)    

    self.frame += 1
    return can_sends