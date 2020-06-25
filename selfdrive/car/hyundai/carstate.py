from cereal import car
from selfdrive.car.hyundai.values import DBC, STEER_THRESHOLD, FEATURES, CAR
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.atom_conf import AtomConf
from selfdrive.car.hyundai.values import Buttons


GearShifter = car.CarState.GearShifter

ATOMC = AtomConf()



class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.cruise_main_button = False
    self.cruise_buttons = False

    self.lkas_button_on = False
    self.lkas_error = False

    self.prev_cruise_main_button = False
    self.prev_cruise_buttons = False

    self.main_on = False
    self.acc_active = False
    self.cruise_engaged_on = False

    self.Mdps_ToiUnavail = 0

    self.left_blinker_flash = 0
    self.right_blinker_flash = 0  
    self.steerWarning = 0  

    self.TSigLHSw = 0
    self.TSigRHSw = 0
    
    #Auto detection for setup
    self.mdps_bus = CP.mdpsBus
    self.sas_bus = CP.sasBus
    self.mdps_error_cnt = 0

  def update(self, cp, cp2, cp_cam):
    global ATOMC
    cp_mdps = cp2 if self.mdps_bus else cp
    cp_sas = cp2 if self.sas_bus else cp

    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_cruise_buttons = self.cruise_buttons

    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CGW1"]['CF_Gway_DrvDrSw'], cp.vl["CGW1"]['CF_Gway_AstDrSw'],
                        cp.vl["CGW2"]['CF_Gway_RLDrSw'], cp.vl["CGW2"]['CF_Gway_RRDrSw']])

    ret.seatbeltUnlatched = cp.vl["CGW1"]['CF_Gway_DrvSeatBeltSw'] == 0

    ret.wheelSpeeds.fl = cp.vl["WHL_SPD11"]['WHL_SPD_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHL_SPD11"]['WHL_SPD_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHL_SPD11"]['WHL_SPD_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHL_SPD11"]['WHL_SPD_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngle = cp_sas.vl["SAS11"]['SAS_Angle']  - ATOMC.steerOffset
    ret.steeringRate = cp_sas.vl["SAS11"]['SAS_Speed']
    ret.yawRate = cp.vl["ESP12"]['YAW_RATE']
    self.TSigLHSw = cp.vl["CGW1"]['CF_Gway_TSigLHSw']
    self.TSigRHSw = cp.vl["CGW1"]['CF_Gway_TSigRHSw']
    leftBlinker = cp.vl["CGW1"]['CF_Gway_TurnSigLh'] != 0
    rightBlinker = cp.vl["CGW1"]['CF_Gway_TurnSigRh'] != 0
    ret.steeringTorque = cp_mdps.vl["MDPS12"]['CR_Mdps_StrColTq']
    ret.steeringTorqueEps = cp_mdps.vl["MDPS12"]['CR_Mdps_OutTq']
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    self.mdps_error_cnt += 1 if cp_mdps.vl["MDPS12"]['CF_Mdps_ToiUnavail'] != 0 else -self.mdps_error_cnt
    ret.steerWarning = self.mdps_error_cnt > 100 #cp_mdps.vl["MDPS12"]['CF_Mdps_ToiUnavail'] != 0

    if leftBlinker:
      self.left_blinker_flash = 60
    elif  self.left_blinker_flash:
      self.left_blinker_flash -= 1

    if rightBlinker:
      self.right_blinker_flash = 60
    elif  self.right_blinker_flash:
      self.right_blinker_flash -= 1

    ret.leftBlinker = self.left_blinker_flash != 0
    ret.rightBlinker = self.right_blinker_flash != 0
    
    self.lead_distance = cp.vl["SCC11"]['ACC_ObjDist']
    lead_objspd = cp.vl["SCC11"]['ACC_ObjRelSpd']
    self.lead_objspd = lead_objspd * CV.MS_TO_KPH

    self.VSetDis = cp.vl["SCC11"]['VSetDis']
    self.clu_Vanz = cp.vl["CLU11"]["CF_Clu_Vanz"]

    self.Mdps_ToiUnavail = cp.vl["MDPS12"]['CF_Mdps_ToiUnavail']
    ret.vEgo = self.clu_Vanz * CV.KPH_TO_MS


    
    steerWarning = False
    if ret.vEgo < 5 or not self.Mdps_ToiUnavail:
      self.steerWarning = 0
    elif self.steerWarning >= 2:
      steerWarning = True
    else:
      self.steerWarning += 1

    ret.steerWarning = steerWarning
    

    # cruise state
    self.main_on = (cp.vl["SCC11"]["MainMode_ACC"] != 0)
    self.acc_active = (cp.vl["SCC12"]['ACCMode'] != 0)

    ret.cruiseState.available = self.main_on
    ret.cruiseState.enabled =  ret.cruiseState.available  #if not self.CP.longcontrolEnabled else ret.cruiseState.enabled
    ret.cruiseState.standstill = cp.vl["SCC11"]['SCCInfoDisplay'] == 4.

    # most HKG cars has no long control, it is safer and easier to engage by main on

    #if ret.cruiseState.enabled:
    if self.acc_active:
      is_set_speed_in_mph = int(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"])
      speed_conv = CV.MPH_TO_MS if is_set_speed_in_mph else CV.KPH_TO_MS
      ret.cruiseState.speed = self.VSetDis * speed_conv
    else:
      ret.cruiseState.speed = 0

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]['DriverBraking'] != 0

    # TODO: Check this
    ret.brakeLights = bool(cp.vl["TCS13"]['BrakeLight'] or ret.brakePressed)

    #TODO: find pedal signal for EV/HYBRID Cars
    ret.gas = cp.vl["EMS12"]['PV_AV_CAN'] / 100 if self.CP.carFingerprint not in FEATURES["use_elect_ems"] else \
                cp.vl["E_EMS11"]['Accel_Pedal_Pos'] / 100

    ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"]) if self.CP.carFingerprint not in FEATURES["use_elect_ems"] else \
                cp.vl["E_EMS11"]['Accel_Pedal_Pos'] > 5

    ret.espDisabled = cp.vl["TCS15"]['ESC_Off_Step'] != 0

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection, as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      if cp.vl["CLU15"]["CF_Clu_InhibitD"] == 1:
        ret.gearShifter = GearShifter.drive
      elif cp.vl["CLU15"]["CF_Clu_InhibitN"] == 1:
        ret.gearShifter = GearShifter.neutral
      elif cp.vl["CLU15"]["CF_Clu_InhibitP"] == 1:
        ret.gearShifter = GearShifter.park
      elif cp.vl["CLU15"]["CF_Clu_InhibitR"] == 1:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.unknown
    # Gear Selecton via TCU12
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
      if gear == 0:
        ret.gearShifter = GearShifter.park
      elif gear == 14:
        ret.gearShifter = GearShifter.reverse
      elif gear > 0 and gear < 9:    # unaware of anything over 8 currently
        ret.gearShifter = GearShifter.drive
      else:
        ret.gearShifter = GearShifter.unknown
    # Gear Selecton - This is only compatible with optima hybrid 2017
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      if gear in (5, 8): # 5: D, 8: sport mode
        ret.gearShifter = GearShifter.drive
      elif gear == 6:
        ret.gearShifter = GearShifter.neutral
      elif gear == 0:
        ret.gearShifter = GearShifter.park
      elif gear == 7:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.unknown
    # Gear Selecton - This is not compatible with all Kia/Hyundai's, But is the best way for those it is compatible with
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]
      if gear in (5, 8): # 5: D, 8: sport mode
        ret.gearShifter = GearShifter.drive
      elif gear == 6:
        ret.gearShifter = GearShifter.neutral
      elif gear == 0:
        ret.gearShifter = GearShifter.park
      elif gear == 7:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.unknown


    # atom append
    self.pcm_acc_status = int(self.acc_active)
    self.driverOverride = cp.vl["TCS13"]["DriverOverride"]     # 1 Acc,  2 bracking, 0 Normal
    self.cruise_main_button = cp.vl["CLU11"]["CF_Clu_CruiseSwMain"]
    self.cruise_buttons = cp.vl["CLU11"]["CF_Clu_CruiseSwState"]         # clu_CruiseSwState


    # save the entire LKAS11, CLU11, SCC12 and MDPS12
    self.lkas11 = cp_cam.vl["LKAS11"]
    self.clu11 = cp.vl["CLU11"]
    self.mdps12 = cp_mdps.vl["MDPS12"]
    self.park_brake = cp.vl["CGW1"]['CF_Gway_ParkBrakeSw']
    self.steer_state = cp_mdps.vl["MDPS12"]['CF_Mdps_ToiActive'] #0 NOT ACTIVE, 1 ACTIVE
    self.cruise_unavail = cp.vl["TCS13"]['CF_VSM_Avail'] != 1
    self.lkas_error = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"] == 7
    #if not self.lkas_error:
    #  self.lkas_button_on = cp_cam.vl["LKAS11"]["CF_Lkas_LdwsSysState"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHL_SPD_FL", "WHL_SPD11", 0),
      ("WHL_SPD_FR", "WHL_SPD11", 0),
      ("WHL_SPD_RL", "WHL_SPD11", 0),
      ("WHL_SPD_RR", "WHL_SPD11", 0),

      ("YAW_RATE", "ESP12", 0),

      ("CF_Gway_DrvSeatBeltInd", "CGW4", 1),

      ("CF_Gway_DrvSeatBeltSw", "CGW1", 0), # Driver Seatbelt
      ("CF_Gway_DrvDrSw", "CGW1", 0),       # Driver Door is open
      ("CF_Gway_AstDrSw", "CGW1", 0),       # Passenger door is open
      ("CF_Gway_RLDrSw", "CGW2", 0),        # Rear reft door is open
      ("CF_Gway_RRDrSw", "CGW2", 0),        # Rear right door is open
      ("CF_Gway_TSigLHSw", "CGW1", 0),
      ("CF_Gway_TurnSigLh", "CGW1", 0),
      ("CF_Gway_TSigRHSw", "CGW1", 0),
      ("CF_Gway_TurnSigRh", "CGW1", 0),
      ("CF_Gway_ParkBrakeSw", "CGW1", 0),   # Parking Brake

      ("CYL_PRES", "ESP12", 0),

      ("CF_Clu_CruiseSwState", "CLU11", 0),
      ("CF_Clu_CruiseSwMain", "CLU11", 0),
      ("CF_Clu_SldMainSW", "CLU11", 0),
      ("CF_Clu_ParityBit1", "CLU11", 0),
      ("CF_Clu_VanzDecimal" , "CLU11", 0),
      ("CF_Clu_Vanz", "CLU11", 0),
      ("CF_Clu_SPEED_UNIT", "CLU11", 0),
      ("CF_Clu_DetentOut", "CLU11", 0),
      ("CF_Clu_RheostatLevel", "CLU11", 0),
      ("CF_Clu_CluInfo", "CLU11", 0),
      ("CF_Clu_AmpInfo", "CLU11", 0),
      ("CF_Clu_AliveCnt1", "CLU11", 0),

      ("ACCEnable", "TCS13", 0),
      ("BrakeLight", "TCS13", 0),
      ("DriverBraking", "TCS13", 0),
      ("DriverOverride", "TCS13", 0),
      ("CF_VSM_Avail", "TCS13", 0),

      ("ESC_Off_Step", "TCS15", 0),

      ("CF_Lvr_GearInf", "LVR11", 0),        # Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

      ("CF_Lca_Stat", "LCA11", 0),
      ("CF_Lca_IndLeft", "LCA11", 0),
      ("CF_Lca_IndRight", "LCA11", 0),

      ("MainMode_ACC", "SCC11", 1),
      ("SCCInfoDisplay", "SCC11", 0),
      ("AliveCounterACC", "SCC11", 0),
      ("VSetDis", "SCC11", 30),
      ("ObjValid", "SCC11", 0),
      ("DriverAlertDisplay", "SCC11", 0),
      ("TauGapSet", "SCC11", 4),
      ("ACC_ObjStatus", "SCC11", 0),
      ("ACC_ObjLatPos", "SCC11", 0),
      ("ACC_ObjDist", "SCC11", 150), #TK211X value is 204.6
      ("ACC_ObjRelSpd", "SCC11", 0),
      ("Navi_SCC_Curve_Status", "SCC11", 0),
      ("Navi_SCC_Curve_Act", "SCC11", 0),
      ("Navi_SCC_Camera_Act", "SCC11", 0),
      ("Navi_SCC_Camera_Status", "SCC11", 2),

      ("ACCMode", "SCC12", 0),
      ("CF_VSM_Prefill", "SCC12", 0),
      ("CF_VSM_DecCmdAct", "SCC12", 0),
      ("CF_VSM_HBACmd", "SCC12", 0),
      ("CF_VSM_Warn", "SCC12", 0),
      ("CF_VSM_Stat", "SCC12", 0),
      ("CF_VSM_BeltCmd", "SCC12", 0),
      ("ACCFailInfo", "SCC12", 0),
      ("ACCMode", "SCC12", 0),
      ("StopReq", "SCC12", 0),
      ("CR_VSM_DecCmd", "SCC12", 0),
      ("aReqRaw", "SCC12", 0), #aReqMax
      ("TakeOverReq", "SCC12", 0),
      ("PreFill", "SCC12", 0),
      ("aReqValue", "SCC12", 0), #aReqMin
      ("CF_VSM_ConfMode", "SCC12", 1),
      ("AEB_Failinfo", "SCC12", 0),
      ("AEB_Status", "SCC12", 2),
      ("AEB_CmdAct", "SCC12", 0),
      ("AEB_StopReq", "SCC12", 0),
      ("CR_VSM_Alive", "SCC12", 0),
      ("CR_VSM_ChkSum", "SCC12", 0),
    ]

    checks = [
      # address, frequency
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
    ]

    if CP.mdpsBus == 0:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12", 0),
        ("CF_Mdps_Def", "MDPS12", 0),
        ("CF_Mdps_ToiActive", "MDPS12", 0),
        ("CF_Mdps_ToiUnavail", "MDPS12", 0),
        ("CF_Mdps_MsgCount2", "MDPS12", 0),
        ("CF_Mdps_Chksum2", "MDPS12", 0),
        ("CF_Mdps_ToiFlt", "MDPS12", 0),
        ("CF_Mdps_SErr", "MDPS12", 0),
        ("CR_Mdps_StrTq", "MDPS12", 0),
        ("CF_Mdps_FailStat", "MDPS12", 0),
        ("CR_Mdps_OutTq", "MDPS12", 0)
      ]
      checks += [
        ("MDPS12", 50)
      ]
    if CP.sasBus == 0:
      signals += [
        ("SAS_Angle", "SAS11", 0),
        ("SAS_Speed", "SAS11", 0),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [
        ("CF_Clu_InhibitD", "CLU15", 0),
        ("CF_Clu_InhibitP", "CLU15", 0),
        ("CF_Clu_InhibitN", "CLU15", 0),
        ("CF_Clu_InhibitR", "CLU15", 0),
      ]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [
        ("CUR_GR", "TCU12",0),
      ]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [
        ("Elect_Gear_Shifter", "ELECT_GEAR", 0),
    ]
    else:
      signals += [
        ("CF_Lvr_Gear","LVR12",0),
      ]
    if CP.carFingerprint not in FEATURES["use_elect_ems"]:
      signals += [
        ("PV_AV_CAN", "EMS12", 0),
        ("CF_Ems_AclAct", "EMS16", 0),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]
    else:
      signals += [
        ("Accel_Pedal_Pos","E_EMS11",0),
        ("Brake_Pedal_Pos","E_EMS11",0),
      ]
      checks += [
        ("E_EMS11", 100),
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_can2_parser(CP):
    signals = []
    checks = []
    if CP.mdpsBus == 1:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12", 0),
        ("CF_Mdps_Def", "MDPS12", 0),
        ("CF_Mdps_ToiActive", "MDPS12", 0),
        ("CF_Mdps_ToiUnavail", "MDPS12", 0),
        ("CF_Mdps_MsgCount2", "MDPS12", 0),
        ("CF_Mdps_Chksum2", "MDPS12", 0),
        ("CF_Mdps_ToiFlt", "MDPS12", 0),
        ("CF_Mdps_SErr", "MDPS12", 0),
        ("CR_Mdps_StrTq", "MDPS12", 0),
        ("CF_Mdps_FailStat", "MDPS12", 0),
        ("CR_Mdps_OutTq", "MDPS12", 0)
      ]
      checks += [
        ("MDPS12", 50)
      ]
    if CP.sasBus == 1:
      signals += [
        ("SAS_Angle", "SAS11", 0),
        ("SAS_Speed", "SAS11", 0),
      ]
      checks += [
        ("SAS11", 100)
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("CF_Lkas_Bca_R", "LKAS11", 0),
      ("CF_Lkas_LdwsSysState", "LKAS11", 0),
      ("CF_Lkas_SysWarning", "LKAS11", 0),
      ("CF_Lkas_LdwsLHWarning", "LKAS11", 0),
      ("CF_Lkas_LdwsRHWarning", "LKAS11", 0),
      ("CF_Lkas_HbaLamp", "LKAS11", 0),
      ("CF_Lkas_FcwBasReq", "LKAS11", 0),
      ("CF_Lkas_ToiFlt", "LKAS11", 0),
      ("CF_Lkas_HbaSysState", "LKAS11", 0),
      ("CF_Lkas_FcwOpt", "LKAS11", 0),
      ("CF_Lkas_HbaOpt", "LKAS11", 0),
      ("CF_Lkas_FcwSysState", "LKAS11", 0),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11", 0),
      ("CF_Lkas_MsgCount", "LKAS11", 0),
      ("CF_Lkas_FusionState", "LKAS11", 0),
      ("CF_Lkas_FcwOpt_USM", "LKAS11", 0),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11", 0)
    ]

    checks = []

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)