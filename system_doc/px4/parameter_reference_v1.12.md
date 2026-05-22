# PX4 Parameter Reference (v1.12.0)

> Source: PX4-Autopilot v1.12.0 (commit a264541861ee876fe4a7d13dad430f69439799a4)
> Generated from: build/px4_sitl_rtps/parameters.json
> Total parameters: 1450, Groups: 52

## Table of Contents

- [Airspeed Validator](#airspeed-validator) (15 params)
- [Attitude Q estimator](#attitude-q-estimator) (9 params)
- [Battery Calibration](#battery-calibration) (26 params)
- [Camera Capture](#camera-capture) (1 params)
- [Camera Control](#camera-control) (3 params)
- [Camera trigger](#camera-trigger) (11 params)
- [Circuit Breaker](#circuit-breaker) (10 params)
- [Commander](#commander) (63 params)
- [Data Link Loss](#data-link-loss) (3 params)
- [EKF2](#ekf2) (119 params)
- [Events](#events) (2 params)
- [FW Attitude Control](#fw-attitude-control) (49 params)
- [FW L1 Control](#fw-l1-control) (27 params)
- [FW Launch detection](#fw-launch-detection) (5 params)
- [FW TECS](#fw-tecs) (24 params)
- [Failure Detector](#failure-detector) (7 params)
- [Follow target](#follow-target) (4 params)
- [GPS](#gps) (8 params)
- [GPS Failure Navigation](#gps-failure-navigation) (4 params)
- [Geofence](#geofence) (6 params)
- [Hover Thrust Estimator](#hover-thrust-estimator) (3 params)
- [Land Detector](#land-detector) (12 params)
- [Landing target Estimator](#landing-target-estimator) (7 params)
- [Local Position Estimator](#local-position-estimator) (39 params)
- [MAVLink](#mavlink) (23 params)
- [Miscellaneous](#miscellaneous) (6 params)
- [Mission](#mission) (22 params)
- [Mixer Output](#mixer-output) (2 params)
- [Mount](#mount) (18 params)
- [Multicopter Attitude Control](#multicopter-attitude-control) (8 params)
- [Multicopter Position Control](#multicopter-position-control) (59 params)
- [Multicopter Rate Control](#multicopter-rate-control) (26 params)
- [PWM Outputs](#pwm-outputs) (197 params)
- [Precision Land](#precision-land) (6 params)
- [Radio Calibration](#radio-calibration) (112 params)
- [Radio Switches](#radio-switches) (28 params)
- [Return Mode](#return-mode) (7 params)
- [Return To Land](#return-to-land) (1 params)
- [Rover Position Control](#rover-position-control) (17 params)
- [Runway Takeoff](#runway-takeoff) (9 params)
- [SD Logging](#sd-logging) (7 params)
- [SITL](#sitl) (2 params)
- [Sensor Calibration](#sensor-calibration) (126 params)
- [Sensors](#sensors) (34 params)
- [System](#system) (17 params)
- [Testing](#testing) (18 params)
- [Thermal Compensation](#thermal-compensation) (171 params)
- [UAVCAN GNSS](#uavcan-gnss) (5 params)
- [UAVCAN Motor Parameters](#uavcan-motor-parameters) (16 params)
- [UUV Attitude Control](#uuv-attitude-control) (11 params)
- [UUV Position Control](#uuv-position-control) (7 params)
- [VTOL Attitude Control](#vtol-attitude-control) (38 params)

---

## Airspeed Validator

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| ASPD_BETA_GATE | Int32 | 1 | 1 | 5 | SD | Airspeed Selector: Gate size for sideslip angle fusion |
| ASPD_BETA_NOISE | Float | 0.3 | 0.0 | 1.0 | rad | Airspeed Selector: Wind estimator sideslip measurement noise |
| ASPD_DO_CHECKS | Int32 | 1 | - | - | - | Enable checks on airspeed sensors |
| ASPD_FALLBACK_GW | Int32 | 0 | - | - | - | Enable fallback to sensor-less airspeed estimation |
| ASPD_FS_INNOV | Float | 1.0 | 0.5 | 3.0 | - | Airspeed failsafe consistency threshold |
| ASPD_FS_INTEG | Float | 5.0 | - | 30.0 | s | Airspeed failsafe consistency delay |
| ASPD_FS_T_START | Int32 | -1 | -1 | 1000 | s | Airspeed failsafe start delay |
| ASPD_FS_T_STOP | Int32 | 2 | 1 | 10 | s | Airspeed failsafe stop delay |
| ASPD_PRIMARY | Int32 | 1 | - | - | - | Index or primary airspeed measurement source |
| ASPD_SCALE | Float | 1.0 | 0.5 | 1.5 | - | Airspeed scale (scale from IAS to CAS) |
| ASPD_SCALE_EST | Int32 | 0 | - | - | - | Automatic airspeed scale estimation on |
| ASPD_SC_P_NOISE | Float | 0.0001 | 0.0 | 0.1 | Hz | Airspeed Selector: Wind estimator true airspeed scale process noise |
| ASPD_TAS_GATE | Int32 | 3 | 1 | 5 | SD | Airspeed Selector: Gate size for true airspeed fusion |
| ASPD_TAS_NOISE | Float | 1.4 | 0.0 | 4.0 | m/s | Airspeed Selector: Wind estimator true airspeed measurement noise |
| ASPD_W_P_NOISE | Float | 0.1 | 0.0 | 1.0 | m/s^2 | Airspeed Selector: Wind estimator wind process noise |

## Attitude Q estimator

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| ATT_ACC_COMP | Int32 | 1 | - | - | - | Acceleration compensation based on GPS velocity |
| ATT_BIAS_MAX | Float | 0.05 | 0.0 | 2.0 | rad/s | Gyro bias limit |
| ATT_EXT_HDG_M | Int32 | 0 | 0 | 2 | - | External heading usage mode (from Motion capture/Vision) |
| ATT_MAG_DECL | Float | 0.0 | - | - | deg | Magnetic declination, in degrees |
| ATT_MAG_DECL_A | Int32 | 1 | - | - | - | Automatic GPS based declination compensation |
| ATT_W_ACC | Float | 0.2 | 0.0 | 1.0 | - | Complimentary filter accelerometer weight |
| ATT_W_EXT_HDG | Float | 0.1 | 0.0 | 1.0 | - | Complimentary filter external heading weight |
| ATT_W_GYRO_BIAS | Float | 0.1 | 0.0 | 1.0 | - | Complimentary filter gyroscope bias weight |
| ATT_W_MAG | Float | 0.1 | 0.0 | 1.0 | - | Complimentary filter magnetometer weight |

## Battery Calibration

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| BAT1_CAPACITY | Float | -1.0 | -1.0 | 100000.0 | mAh | Battery 1 capacity |
| BAT1_N_CELLS | Int32 | 0 | - | - | - | Number of cells for battery 1 |
| BAT1_R_INTERNAL | Float | -1.0 | -1.0 | 0.2 | Ohm | Explicitly defines the per cell internal resistance for battery 1 |
| BAT1_SOURCE | Int32 | 0 | - | - | - | Battery 1 monitoring source |
| BAT1_V_CHARGED | Float | 4.05 | - | - | V | Full cell voltage (5C load) |
| BAT1_V_EMPTY | Float | 3.5 | - | - | V | Empty cell voltage (5C load) |
| BAT1_V_LOAD_DROP | Float | 0.3 | 0.07 | 0.5 | V | Voltage drop per cell on full throttle |
| BAT2_CAPACITY | Float | -1.0 | -1.0 | 100000.0 | mAh | Battery 2 capacity |
| BAT2_N_CELLS | Int32 | 0 | - | - | - | Number of cells for battery 2 |
| BAT2_R_INTERNAL | Float | -1.0 | -1.0 | 0.2 | Ohm | Explicitly defines the per cell internal resistance for battery 2 |
| BAT2_SOURCE | Int32 | -1 | - | - | - | Battery 2 monitoring source |
| BAT2_V_CHARGED | Float | 4.05 | - | - | V | Full cell voltage (5C load) |
| BAT2_V_EMPTY | Float | 3.5 | - | - | V | Empty cell voltage (5C load) |
| BAT2_V_LOAD_DROP | Float | 0.3 | 0.07 | 0.5 | V | Voltage drop per cell on full throttle |
| BAT_A_PER_V | Float | -1.0 | - | - | - | This parameter is deprecated. Please use BAT1_A_PER_V |
| BAT_CAPACITY | Float | -1.0 | -1.0 | 100000.0 | mAh | This parameter is deprecated. Please use BAT1_CAPACITY instead |
| BAT_CRIT_THR | Float | 0.07 | 0.05 | 0.25 | norm | Critical threshold |
| BAT_EMERGEN_THR | Float | 0.05 | 0.03 | 0.1 | norm | Emergency threshold |
| BAT_LOW_THR | Float | 0.15 | 0.12 | 0.5 | norm | Low threshold |
| BAT_N_CELLS | Int32 | 0 | - | - | S | This parameter is deprecated. Please use BAT1_N_CELLS instead |
| BAT_R_INTERNAL | Float | -1.0 | -1.0 | 0.2 | Ohm | This parameter is deprecated. Please use BAT1_R_INTERNAL instead |
| BAT_SOURCE | Int32 | 0 | 0 | 1 | - | This parameter is deprecated. Please use BAT1_SOURCE instead |
| BAT_V_CHARGED | Float | 4.05 | - | - | V | This parameter is deprecated. Please use BAT1_V_CHARGED instead |
| BAT_V_DIV | Float | -1.0 | - | - | - | This parameter is deprecated. Please use BAT1_V_DIV |
| BAT_V_EMPTY | Float | 3.5 | - | - | V | This parameter is deprecated. Please use BAT1_V_EMPTY instead |
| BAT_V_LOAD_DROP | Float | 0.3 | 0.07 | 0.5 | V | This parameter is deprecated. Please use BAT1_V_LOAD_DROP instead |

## Camera Capture

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CAM_CAP_DELAY | Float | 0.0 | 0.0 | 100.0 | ms | Camera strobe delay |

## Camera Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CAM_CAP_EDGE | Int32 | 0 | - | - | - | Camera capture edge |
| CAM_CAP_FBACK | Int32 | 0 | - | - | - | Camera capture feedback |
| CAM_CAP_MODE | Int32 | 0 | - | - | - | Camera capture timestamping mode |

## Camera trigger

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| TRIG_ACT_TIME | Float | 40.0 | 0.1 | 3000.0 | ms | Camera trigger activation time |
| TRIG_DISTANCE | Float | 25.0 | 0.0 | - | m | Camera trigger distance |
| TRIG_INTERFACE | Int32 | 4 | - | - | - | Camera trigger Interface |
| TRIG_INTERVAL | Float | 40.0 | 4.0 | 10000.0 | ms | Camera trigger interval |
| TRIG_MIN_INTERVA | Float | 1.0 | 1.0 | 10000.0 | ms | Minimum camera trigger interval |
| TRIG_MODE | Int32 | 0 | 0 | 4 | - | Camera trigger mode |
| TRIG_PINS | Int32 | 56 | 1 | 12345678 | - | Camera trigger pin |
| TRIG_PINS_EX | Int32 | 0 | 0 | 4294967040 | - | Camera trigger pin extended |
| TRIG_POLARITY | Int32 | 0 | 0 | 1 | - | Camera trigger polarity |
| TRIG_PWM_NEUTRAL | Int32 | 1500 | 1000 | 2000 | us | PWM neutral output on trigger pin |
| TRIG_PWM_SHOOT | Int32 | 1900 | 1000 | 2000 | us | PWM output to trigger shot |

## Circuit Breaker

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CBRK_AIRSPD_CHK | Int32 | 0 | 0 | 162128 | - | Circuit breaker for airspeed sensor |
| CBRK_BUZZER | Int32 | 0 | 0 | 782097 | - | Circuit breaker for disabling buzzer |
| CBRK_ENGINEFAIL | Int32 | 284953 | 0 | 284953 | - | Circuit breaker for engine failure detection |
| CBRK_FLIGHTTERM | Int32 | 121212 | 0 | 121212 | - | Circuit breaker for flight termination |
| CBRK_IO_SAFETY | Int32 | 22027 | 0 | 22027 | - | Circuit breaker for IO safety |
| CBRK_RATE_CTRL | Int32 | 0 | 0 | 140253 | - | Circuit breaker for rate controller output |
| CBRK_SUPPLY_CHK | Int32 | 0 | 0 | 894281 | - | Circuit breaker for power supply check |
| CBRK_USB_CHK | Int32 | 197848 | 0 | 197848 | - | Circuit breaker for USB link check |
| CBRK_VELPOSERR | Int32 | 0 | 0 | 201607 | - | Circuit breaker for position error check |
| CBRK_VTOLARMING | Int32 | 0 | 0 | 159753 | - | Circuit breaker for arming in fixed-wing mode check |

## Commander

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| COM_ARM_ARSP_EN | Int32 | 1 | - | - | - | Enable preflight check for maximal allowed airspeed when arming |
| COM_ARM_AUTH_ID | Int32 | 10 | - | - | - | Arm authorizer system id |
| COM_ARM_AUTH_MET | Int32 | 0 | - | - | - | Arm authorization method |
| COM_ARM_AUTH_REQ | Int32 | 0 | - | - | - | Require arm authorization to arm |
| COM_ARM_AUTH_TO | Float | 1.0 | - | - | s | Arm authorization timeout |
| COM_ARM_CHK_ESCS | Int32 | 0 | - | - | - | Enable checks on ESCs that report telemetry |
| COM_ARM_EKF_HGT | Float | 1.0 | 0.1 | 1.0 | - | Maximum EKF height innovation test ratio that will allow arming |
| COM_ARM_EKF_POS | Float | 0.5 | 0.1 | 1.0 | - | Maximum EKF position innovation test ratio that will allow arming |
| COM_ARM_EKF_VEL | Float | 0.5 | 0.1 | 1.0 | - | Maximum EKF velocity innovation test ratio that will allow arming |
| COM_ARM_EKF_YAW | Float | 0.5 | 0.1 | 1.0 | - | Maximum EKF yaw innovation test ratio that will allow arming |
| COM_ARM_IMU_ACC | Float | 0.7 | 0.1 | 1.0 | m/s^2 | Maximum accelerometer inconsistency between IMU units that will allow arming |
| COM_ARM_IMU_GYR | Float | 0.25 | 0.02 | 0.3 | rad/s | Maximum rate gyro inconsistency between IMU units that will allow arming |
| COM_ARM_MAG_ANG | Int32 | 45 | 3 | 180 | deg | Maximum magnetic field inconsistency between units that will allow arming |
| COM_ARM_MAG_STR | Int32 | 1 | - | - | - | Enable mag strength preflight check |
| COM_ARM_MIS_REQ | Int32 | 0 | - | - | - | Require valid mission to arm |
| COM_ARM_SDCARD | Int32 | 1 | - | - | - | Enable FMU SD card detection check |
| COM_ARM_SWISBTN | Int32 | 0 | - | - | - | Arm switch is a momentary button |
| COM_ARM_WO_GPS | Int32 | 1 | - | - | - | Allow arming without GPS |
| COM_CPU_MAX | Float | 90.0 | -1.0 | 100.0 | % | Maximum allowed CPU load to still arm |
| COM_DISARM_LAND | Float | 2.0 | - | - | s | Time-out for auto disarm after landing |
| COM_DISARM_PRFLT | Float | 10.0 | - | - | s | Time-out for auto disarm if not taking off |
| COM_DL_LOSS_T | Int32 | 10 | 5 | 300 | s | Datalink loss time threshold |
| COM_EF_C2T | Float | 5.0 | 0.0 | 50.0 | A/% | Engine Failure Current/Throttle Threshold |
| COM_EF_THROT | Float | 0.5 | 0.0 | 1.0 | norm | Engine Failure Throttle Threshold |
| COM_EF_TIME | Float | 10.0 | 0.0 | 60.0 | s | Engine Failure Time Threshold |
| COM_FLIGHT_UUID | Int32 | 0 | 0 | - | - | Next flight UUID |
| COM_FLTMODE1 | Int32 | -1 | - | - | - | First flightmode slot (1000-1160) |
| COM_FLTMODE2 | Int32 | -1 | - | - | - | Second flightmode slot (1160-1320) |
| COM_FLTMODE3 | Int32 | -1 | - | - | - | Third flightmode slot (1320-1480) |
| COM_FLTMODE4 | Int32 | -1 | - | - | - | Fourth flightmode slot (1480-1640) |
| COM_FLTMODE5 | Int32 | -1 | - | - | - | Fifth flightmode slot (1640-1800) |
| COM_FLTMODE6 | Int32 | -1 | - | - | - | Sixth flightmode slot (1800-2000) |
| COM_FLT_PROFILE | Int32 | 0 | - | - | - | User Flight Profile |
| COM_HLDL_LOSS_T | Int32 | 120 | 60 | 3600 | s | High Latency Datalink loss time threshold |
| COM_HLDL_REG_T | Int32 | 0 | 0 | 60 | s | High Latency Datalink regain time threshold |
| COM_HOME_H_T | Float | 5.0 | 2.0 | 15.0 | m | Home set horizontal threshold |
| COM_HOME_IN_AIR | Int32 | 0 | - | - | - | Allows setting the home position after takeoff |
| COM_HOME_V_T | Float | 10.0 | 5.0 | 25.0 | m | Home set vertical threshold |
| COM_KILL_DISARM | Float | 5.0 | 0.0 | 30.0 | s | Timeout value for disarming when kill switch is engaged |
| COM_LKDOWN_TKO | Float | 3.0 | -1.0 | 5.0 | s | Timeout for detecting a failure after takeoff |
| COM_LOW_BAT_ACT | Int32 | 0 | - | - | - | Battery failsafe mode |
| COM_MOT_TEST_EN | Int32 | 1 | - | - | - | Enable Motor Testing |
| COM_OBC_LOSS_T | Float | 5.0 | 0.0 | 60.0 | s | Time-out to wait when onboard computer connection is lost before warning about loss connection |
| COM_OBL_ACT | Int32 | 0 | - | - | - | Set offboard loss failsafe mode |
| COM_OBL_RC_ACT | Int32 | 0 | - | - | - | Set offboard loss failsafe mode when RC is available |
| COM_OF_LOSS_T | Float | 1.0 | 0.0 | 60.0 | s | Time-out to wait when offboard connection is lost before triggering offboard lost action |
| COM_POSCTL_NAVL | Int32 | 0 | - | - | - | Position control navigation loss response |
| COM_POS_FS_DELAY | Int32 | 1 | 1 | 100 | s | Loss of position failsafe activation delay |
| COM_POS_FS_EPH | Float | 5.0 | - | - | m | Horizontal position error threshold |
| COM_POS_FS_EPV | Float | 10.0 | - | - | m | Vertical position error threshold |
| COM_POS_FS_GAIN | Int32 | 10 | - | - | - | Loss of position probation gain factor |
| COM_POS_FS_PROB | Int32 | 30 | 1 | 100 | s | Loss of position probation delay at takeoff |
| COM_POWER_COUNT | Int32 | 1 | 0 | 4 | - | Required number of redundant power modules |
| COM_PREARM_MODE | Int32 | 0 | - | - | - | Condition to enter prearmed mode |
| COM_RCL_ACT_T | Float | 15.0 | 0.0 | 25.0 | s | Delay between RC loss and configured reaction |
| COM_RC_ARM_HYST | Int32 | 1000 | 100 | 1500 | ms | RC input arm/disarm command duration |
| COM_RC_IN_MODE | Int32 | 0 | 0 | 2 | - | RC control input mode |
| COM_RC_LOSS_T | Float | 0.5 | 0.0 | 35.0 | s | RC loss time threshold |
| COM_RC_OVERRIDE | Int32 | 1 | 0 | 7 | - | Enable RC stick override of auto and/or offboard modes |
| COM_RC_STICK_OV | Float | 30.0 | 5.0 | 80.0 | % | RC stick override threshold |
| COM_REARM_GRACE | Int32 | 1 | - | - | - | Rearming grace period |
| COM_VEL_FS_EVH | Float | 1.0 | - | - | m/s | Horizontal velocity error threshold |
| RTL_FLT_TIME | Float | 15.0 | - | - | min | Maximum allowed RTL flight in minutes |

## Data Link Loss

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| NAV_AH_ALT | Float | 600.0 | -50.0 | - | m | Airfield home alt |
| NAV_AH_LAT | Int32 | -265847810 | -900000000 | 900000000 | deg*1e7 | Airfield home Lat |
| NAV_AH_LON | Int32 | 1518423250 | -1800000000 | 1800000000 | deg*1e7 | Airfield home Lon |

## EKF2

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| EKF2_ABIAS_INIT | Float | 0.2 | 0.0 | 0.5 | m/s^2 | 1-sigma IMU accelerometer switch-on bias |
| EKF2_ABL_ACCLIM | Float | 25.0 | 20.0 | 200.0 | m/s^2 | Maximum IMU accel magnitude that allows IMU bias learning |
| EKF2_ABL_GYRLIM | Float | 3.0 | 2.0 | 20.0 | rad/s | Maximum IMU gyro angular rate magnitude that allows IMU bias learning |
| EKF2_ABL_LIM | Float | 0.4 | 0.0 | 0.8 | m/s^2 | Accelerometer bias learning limit |
| EKF2_ABL_TAU | Float | 0.5 | 0.1 | 1.0 | s | Time constant used by acceleration and angular rate magnitude checks used to inhibit delta velocity bias learning |
| EKF2_ACC_B_NOISE | Float | 0.003 | 0.0 | 0.01 | m/s^3 | Process noise for IMU accelerometer bias prediction |
| EKF2_ACC_NOISE | Float | 0.35 | 0.01 | 1.0 | m/s^2 | Accelerometer noise for covariance prediction |
| EKF2_AID_MASK | Int32 | 1 | 0 | 511 | - | Integer bitmask controlling data fusion and aiding methods |
| EKF2_ANGERR_INIT | Float | 0.1 | 0.0 | 0.5 | rad | 1-sigma tilt angle uncertainty after gravity vector alignment |
| EKF2_ARSP_THR | Float | 0.0 | 0.0 | - | m/s | Airspeed fusion threshold |
| EKF2_ASPD_MAX | Float | 20.0 | 5.0 | 50.0 | m/s | Upper limit on airspeed along individual axes used to correct baro for position error effects |
| EKF2_ASP_DELAY | Float | 100.0 | 0.0 | 300.0 | ms | Airspeed measurement delay relative to IMU measurements |
| EKF2_AVEL_DELAY | Float | 5.0 | 0.0 | 300.0 | ms | Auxillary Velocity Estimate (e.g from a landing target) delay relative to IMU measurements |
| EKF2_BARO_DELAY | Float | 0.0 | 0.0 | 300.0 | ms | Barometer measurement delay relative to IMU measurements |
| EKF2_BARO_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for barometric and GPS height fusion |
| EKF2_BARO_NOISE | Float | 3.5 | 0.01 | 15.0 | m | Measurement noise for barometric altitude |
| EKF2_BCOEF_X | Float | 25.0 | 1.0 | 100.0 | kg/m^2 | X-axis ballistic coefficient used by the multi-rotor specific drag force model |
| EKF2_BCOEF_Y | Float | 25.0 | 1.0 | 100.0 | kg/m^2 | Y-axis ballistic coefficient used by the multi-rotor specific drag force model |
| EKF2_BETA_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for synthetic sideslip fusion |
| EKF2_BETA_NOISE | Float | 0.3 | 0.1 | 1.0 | m/s | Noise for synthetic sideslip fusion |
| EKF2_DECL_TYPE | Int32 | 7 | 0 | 7 | - | Integer bitmask controlling handling of magnetic declination |
| EKF2_DRAG_NOISE | Float | 2.5 | 0.5 | 10.0 | (m/s^2)^2 | Specific drag force observation noise variance used by the multi-rotor specific drag force model |
| EKF2_EAS_NOISE | Float | 1.4 | 0.5 | 5.0 | m/s | Measurement noise for airspeed fusion |
| EKF2_EVA_NOISE | Float | 0.05 | 0.05 | - | rad | Measurement noise for vision angle observations used to lower bound or replace the uncertainty included in the message |
| EKF2_EVP_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for vision position fusion |
| EKF2_EVP_NOISE | Float | 0.1 | 0.01 | - | m | Measurement noise for vision position observations used to lower bound or replace the uncertainty included in the message |
| EKF2_EVV_GATE | Float | 3.0 | 1.0 | - | SD | Gate size for vision velocity estimate fusion |
| EKF2_EVV_NOISE | Float | 0.1 | 0.01 | - | m/s | Measurement noise for vision velocity observations used to lower bound or replace the uncertainty included in the message |
| EKF2_EV_DELAY | Float | 175.0 | 0.0 | 300.0 | ms | Vision Position Estimator delay relative to IMU measurements |
| EKF2_EV_NOISE_MD | Int32 | 0 | - | - | - | Whether to set the external vision observation noise from the parameter or from vision message |
| EKF2_EV_POS_X | Float | 0.0 | - | - | m | X position of VI sensor focal point in body frame (forward axis with origin relative to vehicle centre of gravity) |
| EKF2_EV_POS_Y | Float | 0.0 | - | - | m | Y position of VI sensor focal point in body frame (right axis with origin relative to vehicle centre of gravity) |
| EKF2_EV_POS_Z | Float | 0.0 | - | - | m | Z position of VI sensor focal point in body frame (down axis with origin relative to vehicle centre of gravity) |
| EKF2_FUSE_BETA | Int32 | 0 | - | - | - | Boolean determining if synthetic sideslip measurements should fused |
| EKF2_GBIAS_INIT | Float | 0.1 | 0.0 | 0.2 | rad/s | 1-sigma IMU gyro switch-on bias |
| EKF2_GND_EFF_DZ | Float | 0.0 | 0.0 | 10.0 | m | Baro deadzone range for height fusion |
| EKF2_GND_MAX_HGT | Float | 0.5 | 0.0 | 5.0 | m | Height above ground level for ground effect zone |
| EKF2_GPS_CHECK | Int32 | 245 | 0 | 511 | - | Integer bitmask controlling GPS checks |
| EKF2_GPS_DELAY | Float | 110.0 | 0.0 | 300.0 | ms | GPS measurement delay relative to IMU measurements |
| EKF2_GPS_POS_X | Float | 0.0 | - | - | m | X position of GPS antenna in body frame (forward axis with origin relative to vehicle centre of gravity) |
| EKF2_GPS_POS_Y | Float | 0.0 | - | - | m | Y position of GPS antenna in body frame (right axis with origin relative to vehicle centre of gravity) |
| EKF2_GPS_POS_Z | Float | 0.0 | - | - | m | Z position of GPS antenna in body frame (down axis with origin relative to vehicle centre of gravity) |
| EKF2_GPS_P_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for GPS horizontal position fusion |
| EKF2_GPS_P_NOISE | Float | 0.5 | 0.01 | 10.0 | m | Measurement noise for gps position |
| EKF2_GPS_V_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for GPS velocity fusion |
| EKF2_GPS_V_NOISE | Float | 0.3 | 0.01 | 5.0 | m/s | Measurement noise for gps horizontal velocity |
| EKF2_GSF_TAS | Float | 15.0 | 0.0 | 100.0 | m/s | Default value of true airspeed used in EKF-GSF AHRS calculation |
| EKF2_GYR_B_NOISE | Float | 0.001 | 0.0 | 0.01 | rad/s^2 | Process noise for IMU rate gyro bias prediction |
| EKF2_GYR_NOISE | Float | 0.015 | 0.0001 | 0.1 | rad/s | Rate gyro noise for covariance prediction |
| EKF2_HDG_GATE | Float | 2.6 | 1.0 | - | SD | Gate size for magnetic heading fusion |
| EKF2_HEAD_NOISE | Float | 0.3 | 0.01 | 1.0 | rad | Measurement noise for magnetic heading fusion |
| EKF2_HGT_MODE | Int32 | 0 | - | - | - | Determines the primary source of height data used by the EKF |
| EKF2_IMU_POS_X | Float | 0.0 | - | - | m | X position of IMU in body frame (forward axis with origin relative to vehicle centre of gravity) |
| EKF2_IMU_POS_Y | Float | 0.0 | - | - | m | Y position of IMU in body frame (right axis with origin relative to vehicle centre of gravity) |
| EKF2_IMU_POS_Z | Float | 0.0 | - | - | m | Z position of IMU in body frame (down axis with origin relative to vehicle centre of gravity) |
| EKF2_MAG_ACCLIM | Float | 0.5 | 0.0 | 5.0 | m/s^2 | Horizontal acceleration threshold used by automatic selection of magnetometer fusion method |
| EKF2_MAG_B_NOISE | Float | 0.0001 | 0.0 | 0.1 | gauss/s | Process noise for body magnetic field prediction |
| EKF2_MAG_CHECK | Int32 | 0 | - | - | - | Magnetic field strength test selection |
| EKF2_MAG_DECL | Float | 0.0 | - | - | deg | Magnetic declination |
| EKF2_MAG_DELAY | Float | 0.0 | 0.0 | 300.0 | ms | Magnetometer measurement delay relative to IMU measurements |
| EKF2_MAG_E_NOISE | Float | 0.001 | 0.0 | 0.1 | gauss/s | Process noise for earth magnetic field prediction |
| EKF2_MAG_GATE | Float | 3.0 | 1.0 | - | SD | Gate size for magnetometer XYZ component fusion |
| EKF2_MAG_NOISE | Float | 0.05 | 0.001 | 1.0 | gauss | Measurement noise for magnetometer 3-axis fusion |
| EKF2_MAG_TYPE | Int32 | 0 | - | - | - | Type of magnetometer fusion |
| EKF2_MAG_YAWLIM | Float | 0.25 | 0.0 | 1.0 | rad/s | Yaw rate threshold used by automatic selection of magnetometer fusion method |
| EKF2_MIN_OBS_DT | Int32 | 20 | 10 | 50 | ms | Minimum time of arrival delta between non-IMU observations before data is downsampled |
| EKF2_MIN_RNG | Float | 0.1 | 0.01 | - | m | Expected range finder reading when on ground |
| EKF2_MOVE_TEST | Float | 1.0 | 0.1 | 10.0 | - | Vehicle movement test threshold |
| EKF2_MULTI_IMU | Int32 | 0 | 0 | 4 | - | Multi-EKF IMUs |
| EKF2_MULTI_MAG | Int32 | 0 | 0 | 4 | - | Multi-EKF Magnetometers |
| EKF2_NOAID_NOISE | Float | 10.0 | 0.5 | 50.0 | m | Measurement noise for non-aiding position hold |
| EKF2_NOAID_TOUT | Int32 | 5000000 | 500000 | 10000000 | us | Maximum lapsed time from last fusion of measurements that constrain velocity drift before the EKF will report the horizontal nav solution as invalid |
| EKF2_OF_DELAY | Float | 20.0 | 0.0 | 300.0 | ms | Optical flow measurement delay relative to IMU measurements |
| EKF2_OF_GATE | Float | 3.0 | 1.0 | - | SD | Gate size for optical flow fusion |
| EKF2_OF_N_MAX | Float | 0.5 | 0.05 | - | rad/s | Measurement noise for the optical flow sensor |
| EKF2_OF_N_MIN | Float | 0.15 | 0.05 | - | rad/s | Measurement noise for the optical flow sensor when it's reported quality metric is at the maximum |
| EKF2_OF_POS_X | Float | 0.0 | - | - | m | X position of optical flow focal point in body frame (forward axis with origin relative to vehicle centre of gravity) |
| EKF2_OF_POS_Y | Float | 0.0 | - | - | m | Y position of optical flow focal point in body frame (right axis with origin relative to vehicle centre of gravity) |
| EKF2_OF_POS_Z | Float | 0.0 | - | - | m | Z position of optical flow focal point in body frame (down axis with origin relative to vehicle centre of gravity) |
| EKF2_OF_QMIN | Int32 | 1 | 0 | 255 | - | Optical Flow data will only be used if the sensor reports a quality metric >= EKF2_OF_QMIN |
| EKF2_PCOEF_XN | Float | 0.0 | -0.5 | 0.5 | - | Static pressure position error coefficient for the negative X axis |
| EKF2_PCOEF_XP | Float | 0.0 | -0.5 | 0.5 | - | Static pressure position error coefficient for the positive X axis |
| EKF2_PCOEF_YN | Float | 0.0 | -0.5 | 0.5 | - | Pressure position error coefficient for the negative Y axis |
| EKF2_PCOEF_YP | Float | 0.0 | -0.5 | 0.5 | - | Pressure position error coefficient for the positive Y axis |
| EKF2_PCOEF_Z | Float | 0.0 | -0.5 | 0.5 | - | Static pressure position error coefficient for the Z axis |
| EKF2_REQ_EPH | Float | 3.0 | 2.0 | 100.0 | m | Required EPH to use GPS |
| EKF2_REQ_EPV | Float | 5.0 | 2.0 | 100.0 | m | Required EPV to use GPS |
| EKF2_REQ_GPS_H | Float | 10.0 | 0.1 | - | s | Required GPS health time on startup |
| EKF2_REQ_HDRIFT | Float | 0.1 | 0.1 | 1.0 | m/s | Maximum horizontal drift speed to use GPS |
| EKF2_REQ_NSATS | Int32 | 6 | 4 | 12 | - | Required satellite count to use GPS |
| EKF2_REQ_PDOP | Float | 2.5 | 1.5 | 5.0 | - | Maximum PDOP to use GPS |
| EKF2_REQ_SACC | Float | 0.5 | 0.5 | 5.0 | m/s | Required speed accuracy to use GPS |
| EKF2_REQ_VDRIFT | Float | 0.2 | 0.1 | 1.5 | m/s | Maximum vertical drift speed to use GPS |
| EKF2_RNG_AID | Int32 | 1 | - | - | - | Range sensor aid |
| EKF2_RNG_A_HMAX | Float | 5.0 | 1.0 | 10.0 | m | Maximum absolute altitude (height above ground level) allowed for range aid mode |
| EKF2_RNG_A_IGATE | Float | 1.0 | 0.1 | 5.0 | SD | Gate size used for innovation consistency checks for range aid fusion |
| EKF2_RNG_A_VMAX | Float | 1.0 | 0.1 | 2.0 | m/s | Maximum horizontal velocity allowed for range aid mode |
| EKF2_RNG_DELAY | Float | 5.0 | 0.0 | 300.0 | ms | Range finder measurement delay relative to IMU measurements |
| EKF2_RNG_GATE | Float | 5.0 | 1.0 | - | SD | Gate size for range finder fusion |
| EKF2_RNG_NOISE | Float | 0.1 | 0.01 | - | m | Measurement noise for range finder fusion |
| EKF2_RNG_PITCH | Float | 0.0 | -0.75 | 0.75 | rad | Range sensor pitch offset |
| EKF2_RNG_POS_X | Float | 0.0 | - | - | m | X position of range finder origin in body frame (forward axis with origin relative to vehicle centre of gravity) |
| EKF2_RNG_POS_Y | Float | 0.0 | - | - | m | Y position of range finder origin in body frame (right axis with origin relative to vehicle centre of gravity) |
| EKF2_RNG_POS_Z | Float | 0.0 | - | - | m | Z position of range finder origin in body frame (down axis with origin relative to vehicle centre of gravity) |
| EKF2_RNG_QLTY_T | Float | 1.0 | 0.1 | 5.0 | s | Minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s) |
| EKF2_RNG_SFE | Float | 0.05 | 0.0 | 0.2 | m/m | Range finder range dependant noise scaler |
| EKF2_SEL_ERR_RED | Float | 0.2 | - | - | - | Selector error reduce threshold |
| EKF2_SEL_IMU_ACC | Float | 1.0 | - | - | m/s^2 | Selector acceleration threshold |
| EKF2_SEL_IMU_ANG | Float | 15.0 | - | - | deg | Selector angular threshold |
| EKF2_SEL_IMU_RAT | Float | 7.0 | - | - | deg/s | Selector angular rate threshold |
| EKF2_SEL_IMU_VEL | Float | 2.0 | - | - | m/s | Selector angular threshold |
| EKF2_SYNT_MAG_Z | Int32 | 0 | - | - | - | Enable synthetic magnetometer Z component measurement |
| EKF2_TAS_GATE | Float | 3.0 | 1.0 | - | SD | Gate size for TAS fusion |
| EKF2_TAU_POS | Float | 0.25 | 0.1 | 1.0 | s | Time constant of the position output prediction and smoothing filter. Controls how tightly the output track the EKF states |
| EKF2_TAU_VEL | Float | 0.25 | - | 1.0 | s | Time constant of the velocity output prediction and smoothing filter |
| EKF2_TERR_GRAD | Float | 0.5 | 0.0 | - | m/m | Magnitude of terrain gradient |
| EKF2_TERR_MASK | Int32 | 3 | 0 | 3 | - | Integer bitmask controlling fusion sources of the terrain estimator |
| EKF2_TERR_NOISE | Float | 5.0 | 0.5 | - | m/s | Terrain altitude process noise - accounts for instability in vehicle height estimate |
| EKF2_WIND_NOISE | Float | 0.1 | 0.0 | 1.0 | m/s^2 | Process noise for wind velocity prediction |

## Events

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| EV_TSK_RC_LOSS | Int32 | 0 | - | - | - | RC Loss Alarm |
| EV_TSK_STAT_DIS | Int32 | 0 | - | - | - | Status Display |

## FW Attitude Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| FW_ACRO_X_MAX | Float | 90.0 | 45.0 | 720.0 | deg | Acro body x max rate |
| FW_ACRO_Y_MAX | Float | 90.0 | 45.0 | 720.0 | deg | Acro body y max rate |
| FW_ACRO_Z_MAX | Float | 45.0 | 10.0 | 180.0 | deg | Acro body z max rate |
| FW_ARSP_MODE | Int32 | 0 | - | - | - | Airspeed mode |
| FW_ARSP_SCALE_EN | Int32 | 1 | - | - | - | Enable airspeed scaling |
| FW_BAT_SCALE_EN | Int32 | 0 | - | - | - | Whether to scale throttle by battery power level |
| FW_DTRIM_P_FLPS | Float | 0.0 | -0.25 | 0.25 | - | Pitch trim increment for flaps configuration |
| FW_DTRIM_P_VMAX | Float | 0.0 | -0.25 | 0.25 | - | Pitch trim increment at maximum airspeed |
| FW_DTRIM_P_VMIN | Float | 0.0 | -0.25 | 0.25 | - | Pitch trim increment at minimum airspeed |
| FW_DTRIM_R_FLPS | Float | 0.0 | -0.25 | 0.25 | - | Roll trim increment for flaps configuration |
| FW_DTRIM_R_VMAX | Float | 0.0 | -0.25 | 0.25 | - | Roll trim increment at maximum airspeed |
| FW_DTRIM_R_VMIN | Float | 0.0 | -0.25 | 0.25 | - | Roll trim increment at minimum airspeed |
| FW_DTRIM_Y_VMAX | Float | 0.0 | -0.25 | 0.25 | - | Yaw trim increment at maximum airspeed |
| FW_DTRIM_Y_VMIN | Float | 0.0 | -0.25 | 0.25 | - | Yaw trim increment at minimum airspeed |
| FW_FLAPERON_SCL | Float | 0.0 | 0.0 | 1.0 | norm | Scale factor for flaperons |
| FW_FLAPS_LND_SCL | Float | 1.0 | 0.0 | 1.0 | norm | Flaps setting during landing |
| FW_FLAPS_SCL | Float | 1.0 | 0.0 | 1.0 | norm | Scale factor for flaps |
| FW_FLAPS_TO_SCL | Float | 0.0 | 0.0 | 1.0 | norm | Flaps setting during take-off |
| FW_MAN_P_MAX | Float | 45.0 | 0.0 | 90.0 | deg | Max manual pitch |
| FW_MAN_P_SC | Float | 1.0 | 0.0 | - | norm | Manual pitch scale |
| FW_MAN_R_MAX | Float | 45.0 | 0.0 | 90.0 | deg | Max manual roll |
| FW_MAN_R_SC | Float | 1.0 | 0.0 | 1.0 | norm | Manual roll scale |
| FW_MAN_Y_SC | Float | 1.0 | 0.0 | - | norm | Manual yaw scale |
| FW_PR_FF | Float | 0.5 | 0.0 | 10.0 | %/rad/s | Pitch rate feed forward |
| FW_PR_I | Float | 0.1 | 0.005 | 0.5 | %/rad | Pitch rate integrator gain |
| FW_PR_IMAX | Float | 0.4 | 0.0 | 1.0 | - | Pitch rate integrator limit |
| FW_PR_P | Float | 0.08 | 0.005 | 1.0 | %/rad/s | Pitch rate proportional gain |
| FW_PSP_OFF | Float | 0.0 | -90.0 | 90.0 | deg | Pitch setpoint offset (pitch at level flight) |
| FW_P_RMAX_NEG | Float | 60.0 | 0.0 | 90.0 | deg/s | Maximum negative / down pitch rate |
| FW_P_RMAX_POS | Float | 60.0 | 0.0 | 90.0 | deg/s | Maximum positive / up pitch rate |
| FW_P_TC | Float | 0.4 | 0.2 | 1.0 | s | Attitude pitch time constant |
| FW_RLL_TO_YAW_FF | Float | 0.0 | 0.0 | - | - | Roll control to yaw control feedforward gain |
| FW_RR_FF | Float | 0.5 | 0.0 | 10.0 | %/rad/s | Roll rate feed forward |
| FW_RR_I | Float | 0.1 | 0.005 | 0.2 | %/rad | Roll rate integrator Gain |
| FW_RR_IMAX | Float | 0.2 | 0.0 | 1.0 | - | Roll integrator anti-windup |
| FW_RR_P | Float | 0.05 | 0.005 | 1.0 | %/rad/s | Roll rate proportional Gain |
| FW_R_RMAX | Float | 70.0 | 0.0 | 90.0 | deg/s | Maximum roll rate |
| FW_R_TC | Float | 0.4 | 0.4 | 1.0 | s | Attitude Roll Time Constant |
| FW_WR_FF | Float | 0.2 | 0.0 | 10.0 | %/rad/s | Wheel steering rate feed forward |
| FW_WR_I | Float | 0.1 | 0.005 | 0.5 | %/rad | Wheel steering rate integrator gain |
| FW_WR_IMAX | Float | 1.0 | 0.0 | 1.0 | - | Wheel steering rate integrator limit |
| FW_WR_P | Float | 0.5 | 0.005 | 1.0 | %/rad/s | Wheel steering rate proportional gain |
| FW_W_EN | Int32 | 0 | - | - | - | Enable wheel steering controller |
| FW_W_RMAX | Float | 30.0 | 0.0 | 90.0 | deg/s | Maximum wheel steering rate |
| FW_YR_FF | Float | 0.3 | 0.0 | 10.0 | %/rad/s | Yaw rate feed forward |
| FW_YR_I | Float | 0.1 | 0.0 | 50.0 | %/rad | Yaw rate integrator gain |
| FW_YR_IMAX | Float | 0.2 | 0.0 | 1.0 | - | Yaw rate integrator limit |
| FW_YR_P | Float | 0.05 | 0.005 | 1.0 | %/rad/s | Yaw rate proportional gain |
| FW_Y_RMAX | Float | 50.0 | 0.0 | 90.0 | deg/s | Maximum yaw rate |

## FW L1 Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| FW_CLMBOUT_DIFF | Float | 10.0 | 0.0 | 150.0 | m | Climbout Altitude difference |
| FW_L1_DAMPING | Float | 0.75 | 0.6 | 0.9 | - | L1 damping |
| FW_L1_PERIOD | Float | 20.0 | 12.0 | 50.0 | m | L1 period |
| FW_L1_R_SLEW_MAX | Float | 90.0 | 0.0 | - | deg/s | L1 controller roll slew rate limit |
| FW_LND_AIRSPD_SC | Float | 1.3 | 1.0 | 1.5 | norm | Min. airspeed scaling factor for landing |
| FW_LND_ANG | Float | 5.0 | 1.0 | 15.0 | deg | Landing slope angle |
| FW_LND_EARLYCFG | Int32 | 0 | - | - | - | Early landing configuration deployment |
| FW_LND_FLALT | Float | 3.0 | 0.0 | 25.0 | m | Landing flare altitude (relative to landing altitude) |
| FW_LND_FL_PMAX | Float | 15.0 | 0.0 | 45.0 | deg | Flare, maximum pitch |
| FW_LND_FL_PMIN | Float | 2.5 | 0.0 | 15.0 | deg | Flare, minimum pitch |
| FW_LND_HHDIST | Float | 15.0 | 0.0 | 30.0 | m | Landing heading hold horizontal distance |
| FW_LND_HVIRT | Float | 10.0 | 1.0 | 15.0 | m | FW_LND_HVIRT |
| FW_LND_THRTC_SC | Float | 1.0 | 0.2 | 1.0 | - | Altitude time constant factor for landing |
| FW_LND_TLALT | Float | -1.0 | -1.0 | 30.0 | m | Landing throttle limit altitude (relative landing altitude) |
| FW_LND_USETER | Int32 | 0 | - | - | - | Use terrain estimate during landing |
| FW_POSCTL_INV_ST | Int32 | 0 | 0 | 1 | - | RC stick mapping fixed-wing |
| FW_P_LIM_MAX | Float | 45.0 | 0.0 | 60.0 | deg | Positive pitch limit |
| FW_P_LIM_MIN | Float | -45.0 | -60.0 | 0.0 | deg | Negative pitch limit |
| FW_R_LIM | Float | 50.0 | 35.0 | 65.0 | deg | Controller roll limit |
| FW_THR_ALT_SCL | Float | 0.0 | 0.0 | 10.0 | - | Scale throttle by pressure change |
| FW_THR_CRUISE | Float | 0.6 | 0.0 | 1.0 | norm | Cruise throttle |
| FW_THR_IDLE | Float | 0.15 | 0.0 | 0.4 | norm | Idle throttle |
| FW_THR_LND_MAX | Float | 1.0 | 0.0 | 1.0 | norm | Throttle limit during landing below throttle limit altitude |
| FW_THR_MAX | Float | 1.0 | 0.0 | 1.0 | norm | Throttle limit max |
| FW_THR_MIN | Float | 0.0 | 0.0 | 1.0 | norm | Throttle limit min |
| FW_THR_SLEW_MAX | Float | 0.0 | 0.0 | 1.0 | - | Throttle max slew rate |
| FW_TKO_PITCH_MIN | Float | 10.0 | -5.0 | 30.0 | deg | Minimum pitch during takeoff |

## FW Launch detection

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| LAUN_ALL_ON | Int32 | 0 | - | - | - | Launch detection |
| LAUN_CAT_A | Float | 30.0 | 0.0 | - | m/s^2 | Catapult accelerometer threshold |
| LAUN_CAT_MDEL | Float | 0.0 | 0.0 | 10.0 | s | Motor delay |
| LAUN_CAT_PMAX | Float | 30.0 | 0.0 | 45.0 | deg | Maximum pitch before the throttle is powered up (during motor delay phase) |
| LAUN_CAT_T | Float | 0.05 | 0.0 | 5.0 | s | Catapult time threshold |

## FW TECS

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| FW_AIRSPD_MAX | Float | 20.0 | 0.5 | 40.0 | m/s | Maximum Airspeed (CAS) |
| FW_AIRSPD_MIN | Float | 10.0 | 0.5 | 40.0 | m/s | Minimum Airspeed (CAS) |
| FW_AIRSPD_STALL | Float | 7.0 | 0.5 | 40.0 | m/s | Stall Airspeed (CAS) |
| FW_AIRSPD_TRIM | Float | 15.0 | 0.5 | 40.0 | m/s | Cruise Airspeed (CAS) |
| FW_GND_SPD_MIN | Float | 5.0 | 0.0 | 40.0 | m/s | Minimum groundspeed |
| FW_T_ALT_TC | Float | 5.0 | 2.0 | - | - | Altitude error time constant |
| FW_T_CLMB_MAX | Float | 5.0 | 1.0 | 15.0 | m/s | Maximum climb rate |
| FW_T_CLMB_R_SP | Float | 3.0 | 0.5 | 15.0 | m/s | Default target climbrate |
| FW_T_HRATE_FF | Float | 0.3 | 0.0 | 1.0 | - | Height rate feed forward |
| FW_T_I_GAIN_PIT | Float | 0.1 | 0.0 | 2.0 | - | Integrator gain pitch |
| FW_T_I_GAIN_THR | Float | 0.3 | 0.0 | 2.0 | - | Integrator gain throttle |
| FW_T_PTCH_DAMP | Float | 0.1 | 0.0 | 2.0 | - | Pitch damping factor |
| FW_T_RLL2THR | Float | 15.0 | 0.0 | 20.0 | - | Roll -> Throttle feedforward |
| FW_T_SEB_R_FF | Float | 1.0 | 0.5 | 3.0 | - | Specific total energy balance rate feedforward gain |
| FW_T_SINK_MAX | Float | 5.0 | 1.0 | 15.0 | m/s | Maximum descent rate |
| FW_T_SINK_MIN | Float | 2.0 | 1.0 | 5.0 | m/s | Minimum descent rate |
| FW_T_SINK_R_SP | Float | 2.0 | 0.5 | 15.0 | m/s | Default target sinkrate |
| FW_T_SPDWEIGHT | Float | 1.0 | 0.0 | 2.0 | - | Speed <--> Altitude priority |
| FW_T_SPD_OMEGA | Float | 2.0 | 1.0 | 10.0 | rad/s | Complementary filter "omega" parameter for speed |
| FW_T_STE_R_TC | Float | 0.4 | 0.0 | 2.0 | - | Specific total energy rate first order filter time constant |
| FW_T_TAS_R_TC | Float | 0.2 | 0.0 | 2.0 | - | True airspeed rate first order filter time constant |
| FW_T_TAS_TC | Float | 5.0 | 2.0 | - | - | True airspeed error time constant |
| FW_T_THR_DAMP | Float | 0.1 | 0.0 | 2.0 | - | Throttle damping factor |
| FW_T_VERT_ACC | Float | 7.0 | 1.0 | 10.0 | m/s^2 | Maximum vertical acceleration |

## Failure Detector

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| FD_ESCS_EN | Int32 | 1 | - | - | - | Enable checks on ESCs that report their arming state |
| FD_EXT_ATS_EN | Int32 | 0 | - | - | - | Enable PWM input on for engaging failsafe from an external automatic trigger system (ATS) |
| FD_EXT_ATS_TRIG | Int32 | 1900 | - | - | us | The PWM threshold from external automatic trigger system for engaging failsafe |
| FD_FAIL_P | Int32 | 60 | 60 | 180 | deg | FailureDetector Max Pitch |
| FD_FAIL_P_TTRI | Float | 0.3 | 0.02 | 5.0 | s | Pitch failure trigger time |
| FD_FAIL_R | Int32 | 60 | 60 | 180 | deg | FailureDetector Max Roll |
| FD_FAIL_R_TTRI | Float | 0.3 | 0.02 | 5.0 | s | Roll failure trigger time |

## Follow target

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| NAV_FT_DST | Float | 8.0 | 1.0 | - | m | Distance to follow target from |
| NAV_FT_FS | Int32 | 1 | 0 | 3 | - | Side to follow target from |
| NAV_FT_RS | Float | 0.5 | 0.0 | 1.0 | - | Dynamic filtering algorithm responsiveness to target movement |
| NAV_MIN_FT_HT | Float | 8.0 | 8.0 | - | m | Minimum follow target altitude |

## GPS

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| GPS_1_GNSS | Int32 | 0 | 0 | 31 | - | GNSS Systems for Primary GPS (integer bitmask) |
| GPS_1_PROTOCOL | Int32 | 1 | 0 | 5 | - | Protocol for Main GPS |
| GPS_2_GNSS | Int32 | 0 | 0 | 31 | - | GNSS Systems for Secondary GPS (integer bitmask) |
| GPS_2_PROTOCOL | Int32 | 1 | 0 | 5 | - | Protocol for Secondary GPS |
| GPS_DUMP_COMM | Int32 | 0 | 0 | 1 | - | Dump GPS communication to a file |
| GPS_UBX_DYNMODEL | Int32 | 7 | 0 | 9 | - | u-blox GPS dynamic platform model |
| GPS_UBX_MODE | Int32 | 0 | 0 | 1 | - | u-blox GPS Mode |
| GPS_YAW_OFFSET | Float | 0.0 | 0.0 | 360.0 | deg | Heading/Yaw offset for dual antenna GPS |

## GPS Failure Navigation

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| NAV_GPSF_LT | Float | 0.0 | 0.0 | 3600.0 | s | Loiter time |
| NAV_GPSF_P | Float | 0.0 | -30.0 | 30.0 | deg | Fixed pitch angle |
| NAV_GPSF_R | Float | 15.0 | 0.0 | 30.0 | deg | Fixed bank angle |
| NAV_GPSF_TR | Float | 0.0 | 0.0 | 1.0 | norm | Thrust |

## Geofence

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| GF_ACTION | Int32 | 2 | 0 | 5 | - | Geofence violation action |
| GF_ALTMODE | Int32 | 0 | 0 | 1 | - | Geofence altitude mode |
| GF_COUNT | Int32 | -1 | -1 | 10 | - | Geofence counter limit |
| GF_MAX_HOR_DIST | Float | 0.0 | 0.0 | 10000.0 | m | Max horizontal distance in meters |
| GF_MAX_VER_DIST | Float | 0.0 | 0.0 | 10000.0 | m | Max vertical distance in meters |
| GF_SOURCE | Int32 | 0 | 0 | 1 | - | Geofence source |

## Hover Thrust Estimator

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| HTE_ACC_GATE | Float | 3.0 | 1.0 | 10.0 | SD | Gate size for acceleration fusion |
| HTE_HT_ERR_INIT | Float | 0.1 | 0.0 | 1.0 | normalized_thrust | 1-sigma initial hover thrust uncertainty |
| HTE_HT_NOISE | Float | 0.0036 | 0.0001 | 1.0 | normalized_thrust/s | Hover thrust process noise |

## Land Detector

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| LNDFW_AIRSPD_MAX | Float | 6.0 | 4.0 | 20.0 | m/s | Airspeed max |
| LNDFW_VEL_XY_MAX | Float | 5.0 | 0.5 | 10.0 | m/s | Fixedwing max horizontal velocity |
| LNDFW_VEL_Z_MAX | Float | 3.0 | 0.1 | 20.0 | m/s | Fixedwing max climb rate |
| LNDFW_XYACC_MAX | Float | 8.0 | 2.0 | 15.0 | m/s^2 | Fixedwing max horizontal acceleration |
| LNDMC_ALT_GND | Float | -1.0 | -1.0 | - | m | Ground effect altitude for multicopters |
| LNDMC_ALT_MAX | Float | -1.0 | -1.0 | 10000.0 | m | Maximum altitude for multicopters |
| LNDMC_ROT_MAX | Float | 20.0 | - | - | deg/s | Multicopter max rotation |
| LNDMC_TRIG_TIME | Float | 1.0 | 0.1 | 10.0 | s | Multicopter land detection trigger time |
| LNDMC_XY_VEL_MAX | Float | 1.5 | - | - | m/s | Multicopter max horizontal velocity |
| LNDMC_Z_VEL_MAX | Float | 0.5 | - | - | m/s | Multicopter max climb rate |
| LND_FLIGHT_T_HI | Int32 | 0 | 0 | - | - | Total flight time in microseconds |
| LND_FLIGHT_T_LO | Int32 | 0 | 0 | - | - | Total flight time in microseconds |

## Landing target Estimator

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| LTEST_ACC_UNC | Float | 10.0 | 0.01 | - | (m/s^2)^2 | Acceleration uncertainty |
| LTEST_MEAS_UNC | Float | 0.005 | - | - | tan(rad)^2 | Landing target measurement uncertainty |
| LTEST_MODE | Int32 | 0 | 0 | 1 | - | Landing target mode |
| LTEST_POS_UNC_IN | Float | 0.1 | 0.001 | - | m^2 | Initial landing target position uncertainty |
| LTEST_SCALE_X | Float | 1.0 | 0.01 | - | - | Scale factor for sensor measurements in sensor x axis |
| LTEST_SCALE_Y | Float | 1.0 | 0.01 | - | - | Scale factor for sensor measurements in sensor y axis |
| LTEST_VEL_UNC_IN | Float | 0.1 | 0.001 | - | (m/s)^2 | Initial landing target velocity uncertainty |

## Local Position Estimator

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| LPE_ACC_XY | Float | 0.012 | 1e-05 | 2.0 | m/s^2/sqrt(Hz) | Accelerometer xy noise density |
| LPE_ACC_Z | Float | 0.02 | 1e-05 | 2.0 | m/s^2/sqrt(Hz) | Accelerometer z noise density |
| LPE_BAR_Z | Float | 3.0 | 0.01 | 100.0 | m | Barometric presssure altitude z standard deviation |
| LPE_EPH_MAX | Float | 3.0 | 1.0 | 5.0 | m | Max EPH allowed for GPS initialization |
| LPE_EPV_MAX | Float | 5.0 | 1.0 | 5.0 | m | Max EPV allowed for GPS initialization |
| LPE_FAKE_ORIGIN | Int32 | 0 | 0 | 1 | - | Enable publishing of a fake global position (e.g for AUTO missions using Optical Flow) |
| LPE_FGYRO_HP | Float | 0.001 | 0.0 | 2.0 | Hz | Flow gyro high pass filter cut off frequency |
| LPE_FLW_OFF_Z | Float | 0.0 | -1.0 | 1.0 | m | Optical flow z offset from center |
| LPE_FLW_QMIN | Int32 | 150 | 0 | 255 | - | Optical flow minimum quality threshold |
| LPE_FLW_R | Float | 7.0 | 0.1 | 10.0 | m/s/rad | Optical flow rotation (roll/pitch) noise gain |
| LPE_FLW_RR | Float | 7.0 | 0.0 | 10.0 | m/rad | Optical flow angular velocity noise gain |
| LPE_FLW_SCALE | Float | 1.3 | 0.1 | 10.0 | m | Optical flow scale |
| LPE_FUSION | Int32 | 145 | 0 | 255 | - | Integer bitmask controlling data fusion |
| LPE_GPS_DELAY | Float | 0.29 | 0.0 | 0.4 | s | GPS delay compensaton |
| LPE_GPS_VXY | Float | 0.25 | 0.01 | 2.0 | m/s | GPS xy velocity standard deviation |
| LPE_GPS_VZ | Float | 0.25 | 0.01 | 2.0 | m/s | GPS z velocity standard deviation |
| LPE_GPS_XY | Float | 1.0 | 0.01 | 5.0 | m | Minimum GPS xy standard deviation, uses reported EPH if greater |
| LPE_GPS_Z | Float | 3.0 | 0.01 | 200.0 | m | Minimum GPS z standard deviation, uses reported EPV if greater |
| LPE_LAND_VXY | Float | 0.05 | 0.01 | 10.0 | m/s | Land detector xy velocity standard deviation |
| LPE_LAND_Z | Float | 0.03 | 0.001 | 10.0 | m | Land detector z standard deviation |
| LPE_LAT | Float | 47.397742 | -90.0 | 90.0 | deg | Local origin latitude for nav w/o GPS |
| LPE_LDR_OFF_Z | Float | 0.0 | -1.0 | 1.0 | m | Lidar z offset from center of vehicle +down |
| LPE_LDR_Z | Float | 0.03 | 0.01 | 1.0 | m | Lidar z standard deviation |
| LPE_LON | Float | 8.545594 | -180.0 | 180.0 | deg | Local origin longitude for nav w/o GPS |
| LPE_LT_COV | Float | 0.0001 | 0.0 | 10.0 | m^2 | Minimum landing target standard covariance, uses reported covariance if greater |
| LPE_PN_B | Float | 0.001 | 0.0 | 1.0 | m/s^3/sqrt(Hz) | Accel bias propagation noise density |
| LPE_PN_P | Float | 0.1 | 0.0 | 1.0 | m/s/sqrt(Hz) | Position propagation noise density |
| LPE_PN_T | Float | 0.001 | 0.0 | 1.0 | m/s/sqrt(Hz) | Terrain random walk noise density, hilly/outdoor (0.1), flat/Indoor (0.001) |
| LPE_PN_V | Float | 0.1 | 0.0 | 1.0 | m/s^2/sqrt(Hz) | Velocity propagation noise density |
| LPE_SNR_OFF_Z | Float | 0.0 | -1.0 | 1.0 | m | Sonar z offset from center of vehicle +down |
| LPE_SNR_Z | Float | 0.05 | 0.01 | 1.0 | m | Sonar z standard deviation |
| LPE_T_MAX_GRADE | Float | 1.0 | 0.0 | 100.0 | % | Terrain maximum percent grade, hilly/outdoor (100 = 45 deg), flat/Indoor (0 = 0 deg) |
| LPE_VIC_P | Float | 0.001 | 0.0001 | 1.0 | m | Vicon position standard deviation |
| LPE_VIS_DELAY | Float | 0.1 | 0.0 | 0.1 | s | Vision delay compensaton |
| LPE_VIS_XY | Float | 0.1 | 0.01 | 1.0 | m | Vision xy standard deviation |
| LPE_VIS_Z | Float | 0.5 | 0.01 | 100.0 | m | Vision z standard deviation |
| LPE_VXY_PUB | Float | 0.3 | 0.01 | 1.0 | m/s | Required velocity xy standard deviation to publish position |
| LPE_X_LP | Float | 5.0 | 5.0 | 1000.0 | Hz | Cut frequency for state publication |
| LPE_Z_PUB | Float | 1.0 | 0.3 | 5.0 | m | Required z standard deviation to publish altitude/ terrain |

## MAVLink

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MAV_0_FORWARD | Int32 | 1 | - | - | - | Enable MAVLink Message forwarding for instance 0 |
| MAV_0_MODE | Int32 | 0 | - | - | - | MAVLink Mode for instance 0 |
| MAV_0_RADIO_CTL | Int32 | 1 | - | - | - | Enable software throttling of mavlink on instance 0 |
| MAV_0_RATE | Int32 | 1200 | 0 | - | B/s | Maximum MAVLink sending rate for instance 0 |
| MAV_1_FORWARD | Int32 | 0 | - | - | - | Enable MAVLink Message forwarding for instance 1 |
| MAV_1_MODE | Int32 | 2 | - | - | - | MAVLink Mode for instance 1 |
| MAV_1_RADIO_CTL | Int32 | 1 | - | - | - | Enable software throttling of mavlink on instance 1 |
| MAV_1_RATE | Int32 | 0 | 0 | - | B/s | Maximum MAVLink sending rate for instance 1 |
| MAV_2_FORWARD | Int32 | 0 | - | - | - | Enable MAVLink Message forwarding for instance 2 |
| MAV_2_MODE | Int32 | 0 | - | - | - | MAVLink Mode for instance 2 |
| MAV_2_RADIO_CTL | Int32 | 1 | - | - | - | Enable software throttling of mavlink on instance 2 |
| MAV_2_RATE | Int32 | 0 | 0 | - | B/s | Maximum MAVLink sending rate for instance 2 |
| MAV_COMP_ID | Int32 | 1 | 1 | 250 | - | MAVLink component ID |
| MAV_FWDEXTSP | Int32 | 1 | - | - | - | Forward external setpoint messages |
| MAV_HASH_CHK_EN | Int32 | 1 | - | - | - | Parameter hash check |
| MAV_HB_FORW_EN | Int32 | 1 | - | - | - | Hearbeat message forwarding |
| MAV_ODOM_LP | Int32 | 0 | - | - | - | Activate ODOMETRY loopback |
| MAV_PROTO_VER | Int32 | 0 | - | - | - | MAVLink protocol version |
| MAV_RADIO_TOUT | Int32 | 5 | 1 | 250 | s | Timeout in seconds for the RADIO_STATUS reports coming in |
| MAV_SIK_RADIO_ID | Int32 | 0 | -1 | 240 | - | MAVLink SiK Radio ID |
| MAV_SYS_ID | Int32 | 1 | 1 | 250 | - | MAVLink system ID |
| MAV_TYPE | Int32 | 2 | 1 | 27 | - | MAVLink airframe type |
| MAV_USEHILGPS | Int32 | 0 | - | - | - | Use/Accept HIL GPS message even if not in HIL mode |

## Miscellaneous

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| EXFW_HDNG_P | Float | 0.1 | - | - | - | EXFW_HDNG_P |
| EXFW_PITCH_P | Float | 0.2 | - | - | - | EXFW_PITCH_P |
| EXFW_ROLL_P | Float | 0.2 | - | - | - | EXFW_ROLL_P |
| MPC_LAND_RC_HELP | Int32 | 0 | 0 | 1 | - | Enable user assisted descent speed for autonomous land routine |
| RV_YAW_P | Float | 0.1 | - | - | - | RV_YAW_P |
| UUV_SKIP_CTRL | Int32 | 0 | - | - | - | Skip the controller |

## Mission

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| COM_OBS_AVOID | Int32 | 0 | - | - | - | Flag to enable obstacle avoidance |
| COM_TAKEOFF_ACT | Int32 | 0 | - | - | - | Action after TAKEOFF has been accepted |
| MIS_DIST_1WP | Float | 900.0 | 0.0 | 10000.0 | m | Maximal horizontal distance from home to first waypoint |
| MIS_DIST_WPS | Float | 900.0 | 0.0 | 10000.0 | m | Maximal horizontal distance between waypoint |
| MIS_LTRMIN_ALT | Float | -1.0 | -1.0 | 80.0 | m | Minimum Loiter altitude |
| MIS_MNT_YAW_CTL | Int32 | 0 | 0 | 1 | - | Enable yaw control of the mount. (Only affects multicopters and ROI mission items) |
| MIS_TAKEOFF_ALT | Float | 2.5 | 0.0 | 80.0 | m | Take-off altitude |
| MIS_TAKEOFF_REQ | Int32 | 0 | - | - | - | Take-off waypoint required |
| MIS_YAW_ERR | Float | 12.0 | 0.0 | 90.0 | deg | Max yaw error in degrees needed for waypoint heading acceptance |
| MIS_YAW_TMT | Float | -1.0 | -1.0 | 20.0 | s | Time in seconds we wait on reaching target heading at a waypoint if it is forced |
| MPC_YAW_MODE | Int32 | 0 | 0 | 4 | - | Yaw mode |
| NAV_ACC_RAD | Float | 10.0 | 0.05 | 200.0 | m | Acceptance Radius |
| NAV_DLL_ACT | Int32 | 0 | - | - | - | Set data link loss failsafe mode |
| NAV_FORCE_VT | Int32 | 1 | - | - | - | Force VTOL mode takeoff and land |
| NAV_FW_ALTL_RAD | Float | 5.0 | 0.05 | 200.0 | m | FW Altitude Acceptance Radius before a landing |
| NAV_FW_ALT_RAD | Float | 10.0 | 0.05 | 200.0 | m | FW Altitude Acceptance Radius |
| NAV_LOITER_RAD | Float | 50.0 | 25.0 | 1000.0 | m | Loiter radius (FW only) |
| NAV_MC_ALT_RAD | Float | 0.8 | 0.05 | 200.0 | m | MC Altitude Acceptance Radius |
| NAV_RCL_ACT | Int32 | 2 | - | - | - | Set RC loss failsafe mode |
| NAV_TRAFF_AVOID | Int32 | 1 | - | - | - | Set traffic avoidance mode |
| NAV_TRAFF_A_RADM | Float | 500.0 | 500.0 | - | m | Set NAV TRAFFIC AVOID RADIUS MANNED |
| NAV_TRAFF_A_RADU | Float | 10.0 | 10.0 | 500.0 | m | Set NAV TRAFFIC AVOID RADIUS |

## Mixer Output

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MC_AIRMODE | Int32 | 0 | - | - | - | Multicopter air-mode |
| MOT_ORDERING | Int32 | 0 | - | - | - | Motor Ordering |

## Mount

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MNT_DO_STAB | Int32 | 0 | 0 | 2 | - | Stabilize the mount |
| MNT_MAN_PITCH | Int32 | 0 | 0 | 6 | - | Auxiliary channel to control pitch (in AUX input or manual mode) |
| MNT_MAN_ROLL | Int32 | 0 | 0 | 6 | - | Auxiliary channel to control roll (in AUX input or manual mode) |
| MNT_MAN_YAW | Int32 | 0 | 0 | 6 | - | Auxiliary channel to control yaw (in AUX input or manual mode) |
| MNT_MAV_COMPID | Int32 | 154 | - | - | - | Mavlink Component ID of the mount |
| MNT_MAV_SYSID | Int32 | 1 | - | - | - | Mavlink System ID of the mount |
| MNT_MODE_IN | Int32 | -1 | -1 | 4 | - | Mount input mode |
| MNT_MODE_OUT | Int32 | 0 | 0 | 2 | - | Mount output mode |
| MNT_OB_LOCK_MODE | Float | 0.0 | -1.0 | 1.0 | - | Mixer value for selecting a locking mode |
| MNT_OB_NORM_MODE | Float | -1.0 | -1.0 | 1.0 | - | Mixer value for selecting normal mode |
| MNT_OFF_PITCH | Float | 0.0 | -360.0 | 360.0 | - | Offset for pitch channel output in degrees |
| MNT_OFF_ROLL | Float | 0.0 | -360.0 | 360.0 | - | Offset for roll channel output in degrees |
| MNT_OFF_YAW | Float | 0.0 | -360.0 | 360.0 | - | Offset for yaw channel output in degrees |
| MNT_RANGE_PITCH | Float | 360.0 | 1.0 | 720.0 | - | Range of pitch channel output in degrees (only in AUX output mode) |
| MNT_RANGE_ROLL | Float | 360.0 | 1.0 | 720.0 | - | Range of roll channel output in degrees (only in AUX output mode) |
| MNT_RANGE_YAW | Float | 360.0 | 1.0 | 720.0 | - | Range of yaw channel output in degrees (only in AUX output mode) |
| MNT_RATE_PITCH | Float | 30.0 | 1.0 | 90.0 | - | Angular pitch rate for manual input in degrees/second |
| MNT_RATE_YAW | Float | 30.0 | 1.0 | 90.0 | - | Angular yaw rate for manual input in degrees/second |

## Multicopter Attitude Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MC_PITCHRATE_MAX | Float | 220.0 | 0.0 | 1800.0 | deg/s | Max pitch rate |
| MC_PITCH_P | Float | 6.5 | 0.0 | 12.0 | - | Pitch P gain |
| MC_ROLLRATE_MAX | Float | 220.0 | 0.0 | 1800.0 | deg/s | Max roll rate |
| MC_ROLL_P | Float | 6.5 | 0.0 | 12.0 | - | Roll P gain |
| MC_YAWRATE_MAX | Float | 200.0 | 0.0 | 1800.0 | deg/s | Max yaw rate |
| MC_YAW_P | Float | 2.8 | 0.0 | 5.0 | - | Yaw P gain |
| MC_YAW_WEIGHT | Float | 0.4 | 0.0 | 1.0 | - | Yaw weight |
| MPC_YAWRAUTO_MAX | Float | 45.0 | 0.0 | 360.0 | deg/s | Max yaw rate in auto mode |

## Multicopter Position Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CP_DELAY | Float | 0.4 | 0.0 | 1.0 | s | Average delay of the range sensor message plus the tracking delay of the position controller in seconds |
| CP_DIST | Float | -1.0 | -1.0 | 15.0 | m | Minimum distance the vehicle should keep to all obstacles |
| CP_GO_NO_DATA | Int32 | 0 | - | - | - | Boolean to allow moving into directions where there is no sensor data (outside FOV) |
| CP_GUIDE_ANG | Float | 30.0 | 0.0 | 90.0 | deg | Angle left/right from the commanded setpoint by which the collision prevention algorithm can choose to change the setpoint direction |
| MC_MAN_TILT_TAU | Float | 0.0 | 0.0 | 2.0 | s | Manual tilt input filter time constant |
| MPC_ACC_DOWN_MAX | Float | 3.0 | 2.0 | 15.0 | m/s^2 | Maximum vertical acceleration in velocity controlled modes down |
| MPC_ACC_HOR | Float | 3.0 | 2.0 | 15.0 | m/s^2 | Acceleration for auto and for manual |
| MPC_ACC_HOR_MAX | Float | 5.0 | 2.0 | 15.0 | m/s^2 | Maximum horizontal acceleration for auto mode and for manual mode |
| MPC_ACC_UP_MAX | Float | 4.0 | 2.0 | 15.0 | m/s^2 | Maximum vertical acceleration in velocity controlled modes upward |
| MPC_ALT_MODE | Int32 | 0 | 0 | 2 | - | Altitude control mode |
| MPC_HOLD_DZ | Float | 0.1 | 0.0 | 1.0 | - | Deadzone of sticks where position hold is enabled |
| MPC_HOLD_MAX_XY | Float | 0.8 | 0.0 | 3.0 | m/s | Maximum horizontal velocity for which position hold is enabled (use 0 to disable check) |
| MPC_HOLD_MAX_Z | Float | 0.6 | 0.0 | 3.0 | m/s | Maximum vertical velocity for which position hold is enabled (use 0 to disable check) |
| MPC_JERK_AUTO | Float | 4.0 | 1.0 | 80.0 | m/s^3 | Jerk limit in auto mode |
| MPC_JERK_MAX | Float | 8.0 | 0.5 | 500.0 | m/s^3 | Maximum jerk limit |
| MPC_LAND_ALT1 | Float | 10.0 | 0.0 | 122.0 | m | Altitude for 1. step of slow landing (descend) |
| MPC_LAND_ALT2 | Float | 5.0 | 0.0 | 122.0 | m | Altitude for 2. step of slow landing (landing) |
| MPC_LAND_SPEED | Float | 0.7 | 0.6 | - | m/s | Landing descend rate |
| MPC_LAND_VEL_XY | Float | 10.0 | 0.0 | - | m/s | Maximum horizontal position mode velocity when close to ground/home altitude |
| MPC_MANTHR_MIN | Float | 0.08 | 0.0 | 1.0 | norm | Minimum manual thrust |
| MPC_MAN_TILT_MAX | Float | 35.0 | 0.0 | 90.0 | deg | Maximal tilt angle in manual or altitude mode |
| MPC_MAN_Y_MAX | Float | 150.0 | 0.0 | 400.0 | deg/s | Max manual yaw rate |
| MPC_MAN_Y_TAU | Float | 0.08 | 0.0 | 5.0 | s | Manual yaw rate input filter time constant |
| MPC_POS_MODE | Int32 | 4 | - | - | - | Manual-Position control sub-mode |
| MPC_SPOOLUP_TIME | Float | 1.0 | 0.0 | 10.0 | s | Enforced delay between arming and takeoff |
| MPC_THR_CURVE | Int32 | 0 | - | - | - | Thrust curve in Manual Mode |
| MPC_THR_HOVER | Float | 0.5 | 0.1 | 0.8 | norm | Hover thrust |
| MPC_THR_MAX | Float | 1.0 | 0.0 | 1.0 | norm | Maximum thrust in auto thrust control |
| MPC_THR_MIN | Float | 0.12 | 0.05 | 1.0 | norm | Minimum thrust in auto thrust control |
| MPC_TILTMAX_AIR | Float | 45.0 | 20.0 | 89.0 | deg | Maximum tilt angle in air |
| MPC_TILTMAX_LND | Float | 12.0 | 10.0 | 89.0 | deg | Maximum tilt during landing |
| MPC_TKO_RAMP_T | Float | 3.0 | 0.0 | 5.0 | - | Position control smooth takeoff ramp time constant |
| MPC_TKO_SPEED | Float | 1.5 | 1.0 | 5.0 | m/s | Takeoff climb rate |
| MPC_USE_HTE | Int32 | 1 | - | - | - | Hover thrust source selector |
| MPC_VELD_LP | Float | 5.0 | 0.0 | 10.0 | Hz | Low pass filter cut freq. for numerical velocity derivative |
| MPC_VEL_MANUAL | Float | 10.0 | 3.0 | 20.0 | m/s | Maximum horizontal velocity setpoint for manual controlled mode |
| MPC_XY_CRUISE | Float | 5.0 | 3.0 | 20.0 | m/s | Maximum horizontal velocity in mission |
| MPC_XY_ERR_MAX | Float | 2.0 | 0.1 | 10.0 | - | Maximum horizontal error allowed by the trajectory generator |
| MPC_XY_MAN_EXPO | Float | 0.6 | 0.0 | 1.0 | - | Manual position control stick exponential curve sensitivity |
| MPC_XY_P | Float | 0.95 | 0.0 | 2.0 | - | Proportional gain for horizontal position error |
| MPC_XY_TRAJ_P | Float | 0.5 | 0.1 | 1.0 | - | Proportional gain for horizontal trajectory position error |
| MPC_XY_VEL_ALL | Float | -10.0 | -20.0 | 20.0 | - | Overall Horizonal Velocity Limit |
| MPC_XY_VEL_D_ACC | Float | 0.2 | 0.1 | 2.0 | - | Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again |
| MPC_XY_VEL_I_ACC | Float | 0.4 | 0.0 | 60.0 | - | Integral gain for horizontal velocity error |
| MPC_XY_VEL_MAX | Float | 12.0 | 0.0 | 20.0 | m/s | Maximum horizontal velocity |
| MPC_XY_VEL_P_ACC | Float | 1.8 | 1.2 | 5.0 | - | Proportional gain for horizontal velocity error |
| MPC_YAW_EXPO | Float | 0.6 | 0.0 | 1.0 | - | Manual control stick yaw rotation exponential curve |
| MPC_Z_MAN_EXPO | Float | 0.6 | 0.0 | 1.0 | - | Manual control stick vertical exponential curve |
| MPC_Z_P | Float | 1.0 | 0.0 | 1.5 | - | Proportional gain for vertical position error |
| MPC_Z_VEL_ALL | Float | -3.0 | -3.0 | 8.0 | - | Overall Vertical Velocity Limit |
| MPC_Z_VEL_D_ACC | Float | 0.0 | 0.0 | 2.0 | - | Differential gain for vertical velocity error |
| MPC_Z_VEL_I_ACC | Float | 2.0 | 0.2 | 3.0 | - | Integral gain for vertical velocity error |
| MPC_Z_VEL_MAX_DN | Float | 1.0 | 0.5 | 4.0 | m/s | Maximum vertical descent velocity |
| MPC_Z_VEL_MAX_UP | Float | 3.0 | 0.5 | 8.0 | m/s | Maximum vertical ascent velocity |
| MPC_Z_VEL_P_ACC | Float | 4.0 | 2.0 | 15.0 | - | Proportional gain for vertical velocity error |
| SYS_VEHICLE_RESP | Float | -0.4 | -1.0 | 1.0 | - | Responsiveness |
| WV_EN | Int32 | 0 | - | - | - | Enable weathervane |
| WV_ROLL_MIN | Float | 1.0 | 0.0 | 5.0 | deg | Minimum roll angle setpoint for weathervane controller to demand a yaw-rate |
| WV_YRATE_MAX | Float | 90.0 | 0.0 | 120.0 | deg/s | Maximum yawrate the weathervane controller is allowed to demand |

## Multicopter Rate Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MC_ACRO_EXPO | Float | 0.69 | 0.0 | 1.0 | - | Acro mode Expo factor for Roll and Pitch |
| MC_ACRO_EXPO_Y | Float | 0.69 | 0.0 | 1.0 | - | Acro mode Expo factor for Yaw |
| MC_ACRO_P_MAX | Float | 720.0 | 0.0 | 1800.0 | deg/s | Max acro pitch rate |
| MC_ACRO_R_MAX | Float | 720.0 | 0.0 | 1800.0 | deg/s | Max acro roll rate |
| MC_ACRO_SUPEXPO | Float | 0.7 | 0.0 | 0.95 | - | Acro mode SuperExpo factor for Roll and Pitch |
| MC_ACRO_SUPEXPOY | Float | 0.7 | 0.0 | 0.95 | - | Acro mode SuperExpo factor for Yaw |
| MC_ACRO_Y_MAX | Float | 540.0 | 0.0 | 1800.0 | deg/s | Max acro yaw rate |
| MC_BAT_SCALE_EN | Int32 | 0 | - | - | - | Battery power level scaler |
| MC_PITCHRATE_D | Float | 0.003 | 0.0 | - | - | Pitch rate D gain |
| MC_PITCHRATE_FF | Float | 0.0 | 0.0 | - | - | Pitch rate feedforward |
| MC_PITCHRATE_I | Float | 0.2 | 0.0 | - | - | Pitch rate I gain |
| MC_PITCHRATE_K | Float | 1.0 | 0.01 | 5.0 | - | Pitch rate controller gain |
| MC_PITCHRATE_P | Float | 0.15 | 0.01 | 0.6 | - | Pitch rate P gain |
| MC_PR_INT_LIM | Float | 0.3 | 0.0 | - | - | Pitch rate integrator limit |
| MC_ROLLRATE_D | Float | 0.003 | 0.0 | 0.01 | - | Roll rate D gain |
| MC_ROLLRATE_FF | Float | 0.0 | 0.0 | - | - | Roll rate feedforward |
| MC_ROLLRATE_I | Float | 0.2 | 0.0 | - | - | Roll rate I gain |
| MC_ROLLRATE_K | Float | 1.0 | 0.01 | 5.0 | - | Roll rate controller gain |
| MC_ROLLRATE_P | Float | 0.15 | 0.01 | 0.5 | - | Roll rate P gain |
| MC_RR_INT_LIM | Float | 0.3 | 0.0 | - | - | Roll rate integrator limit |
| MC_YAWRATE_D | Float | 0.0 | 0.0 | - | - | Yaw rate D gain |
| MC_YAWRATE_FF | Float | 0.0 | 0.0 | - | - | Yaw rate feedforward |
| MC_YAWRATE_I | Float | 0.1 | 0.0 | - | - | Yaw rate I gain |
| MC_YAWRATE_K | Float | 1.0 | 0.0 | 5.0 | - | Yaw rate controller gain |
| MC_YAWRATE_P | Float | 0.2 | 0.0 | 0.6 | - | Yaw rate P gain |
| MC_YR_INT_LIM | Float | 0.3 | 0.0 | - | - | Yaw rate integrator limit |

## PWM Outputs

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| MOT_SLEW_MAX | Float | 0.0 | 0.0 | - | s/(1000*PWM) | Minimum motor rise time (slew rate limit) |
| PWM_AUX_DIS1 | Int32 | -1 | -1 | 2150 | us | PWM aux 1 disarmed value |
| PWM_AUX_DIS2 | Int32 | -1 | -1 | 2150 | us | PWM aux 2 disarmed value |
| PWM_AUX_DIS3 | Int32 | -1 | -1 | 2150 | us | PWM aux 3 disarmed value |
| PWM_AUX_DIS4 | Int32 | -1 | -1 | 2150 | us | PWM aux 4 disarmed value |
| PWM_AUX_DIS5 | Int32 | -1 | -1 | 2150 | us | PWM aux 5 disarmed value |
| PWM_AUX_DIS6 | Int32 | -1 | -1 | 2150 | us | PWM aux 6 disarmed value |
| PWM_AUX_DIS7 | Int32 | -1 | -1 | 2150 | us | PWM aux 7 disarmed value |
| PWM_AUX_DIS8 | Int32 | -1 | -1 | 2150 | us | PWM aux 8 disarmed value |
| PWM_AUX_DISARM | Int32 | 1500 | 0 | 2200 | us | PWM aux disarmed value |
| PWM_AUX_FAIL1 | Int32 | -1 | -1 | 2150 | us | PWM aux 1 failsafe value |
| PWM_AUX_FAIL2 | Int32 | -1 | -1 | 2150 | us | PWM aux 2 failsafe value |
| PWM_AUX_FAIL3 | Int32 | -1 | -1 | 2150 | us | PWM aux 3 failsafe value |
| PWM_AUX_FAIL4 | Int32 | -1 | -1 | 2150 | us | PWM aux 4 failsafe value |
| PWM_AUX_FAIL5 | Int32 | -1 | -1 | 2150 | us | PWM aux 5 failsafe value |
| PWM_AUX_FAIL6 | Int32 | -1 | -1 | 2150 | us | PWM aux 6 failsafe value |
| PWM_AUX_FAIL7 | Int32 | -1 | -1 | 2150 | us | PWM aux 7 failsafe value |
| PWM_AUX_FAIL8 | Int32 | -1 | -1 | 2150 | us | PWM aux 8 failsafe value |
| PWM_AUX_MAX | Int32 | 2000 | 1600 | 2200 | us | PWM aux maximum value |
| PWM_AUX_MAX1 | Int32 | -1 | -1 | 2150 | us | PWM aux 1 maximum value |
| PWM_AUX_MAX2 | Int32 | -1 | -1 | 2150 | us | PWM aux 2 maximum value |
| PWM_AUX_MAX3 | Int32 | -1 | -1 | 2150 | us | PWM aux 3 maximum value |
| PWM_AUX_MAX4 | Int32 | -1 | -1 | 2150 | us | PWM aux 4 maximum value |
| PWM_AUX_MAX5 | Int32 | -1 | -1 | 2150 | us | PWM aux 5 maximum value |
| PWM_AUX_MAX6 | Int32 | -1 | -1 | 2150 | us | PWM aux 6 maximum value |
| PWM_AUX_MAX7 | Int32 | -1 | -1 | 2150 | us | PWM aux 7 maximum value |
| PWM_AUX_MAX8 | Int32 | -1 | -1 | 2150 | us | PWM aux 8 maximum value |
| PWM_AUX_MIN | Int32 | 1000 | 800 | 1400 | us | PWM aux minimum value |
| PWM_AUX_MIN1 | Int32 | -1 | -1 | 1600 | us | PWM aux 1 minimum value |
| PWM_AUX_MIN2 | Int32 | -1 | -1 | 1600 | us | PWM aux 2 minimum value |
| PWM_AUX_MIN3 | Int32 | -1 | -1 | 1600 | us | PWM aux 3 minimum value |
| PWM_AUX_MIN4 | Int32 | -1 | -1 | 1600 | us | PWM aux 4 minimum value |
| PWM_AUX_MIN5 | Int32 | -1 | -1 | 1600 | us | PWM aux 5 minimum value |
| PWM_AUX_MIN6 | Int32 | -1 | -1 | 1600 | us | PWM aux 6 minimum value |
| PWM_AUX_MIN7 | Int32 | -1 | -1 | 1600 | us | PWM aux 7 minimum value |
| PWM_AUX_MIN8 | Int32 | -1 | -1 | 1600 | us | PWM aux 8 minimum value |
| PWM_AUX_RATE | Int32 | 50 | -1 | 2000 | Hz | PWM aux output frequency |
| PWM_AUX_RATE1 | Int32 | 50 | 0 | 400 | Hz | PWM aux 1 rate |
| PWM_AUX_REV1 | Int32 | 0 | - | - | - | PWM aux 1 reverse value |
| PWM_AUX_REV2 | Int32 | 0 | - | - | - | PWM aux 2 reverse value |
| PWM_AUX_REV3 | Int32 | 0 | - | - | - | PWM aux 3 reverse value |
| PWM_AUX_REV4 | Int32 | 0 | - | - | - | PWM aux 4 reverse value |
| PWM_AUX_REV5 | Int32 | 0 | - | - | - | PWM aux 5 reverse value |
| PWM_AUX_REV6 | Int32 | 0 | - | - | - | PWM aux 6 reverse value |
| PWM_AUX_REV7 | Int32 | 0 | - | - | - | PWM aux 7 reverse value |
| PWM_AUX_REV8 | Int32 | 0 | - | - | - | PWM aux 8 reverse value |
| PWM_AUX_TRIM1 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 1 trim value |
| PWM_AUX_TRIM2 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 2 trim value |
| PWM_AUX_TRIM3 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 3 trim value |
| PWM_AUX_TRIM4 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 4 trim value |
| PWM_AUX_TRIM5 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 5 trim value |
| PWM_AUX_TRIM6 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 6 trim value |
| PWM_AUX_TRIM7 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 7 trim value |
| PWM_AUX_TRIM8 | Float | 0.0 | -0.2 | 0.2 | - | PWM aux 8 trim value |
| PWM_EXTRA_DIS1 | Int32 | -1 | -1 | 2150 | us | PWM extra 1 disarmed value |
| PWM_EXTRA_DIS2 | Int32 | -1 | -1 | 2150 | us | PWM extra 2 disarmed value |
| PWM_EXTRA_DIS3 | Int32 | -1 | -1 | 2150 | us | PWM extra 3 disarmed value |
| PWM_EXTRA_DIS4 | Int32 | -1 | -1 | 2150 | us | PWM extra 4 disarmed value |
| PWM_EXTRA_DIS5 | Int32 | -1 | -1 | 2150 | us | PWM extra 5 disarmed value |
| PWM_EXTRA_DIS6 | Int32 | -1 | -1 | 2150 | us | PWM extra 6 disarmed value |
| PWM_EXTRA_DIS7 | Int32 | -1 | -1 | 2150 | us | PWM extra 7 disarmed value |
| PWM_EXTRA_DIS8 | Int32 | -1 | -1 | 2150 | us | PWM extra 8 disarmed value |
| PWM_EXTRA_DISARM | Int32 | 1500 | 0 | 2200 | us | PWM extra disarmed value |
| PWM_EXTRA_FAIL1 | Int32 | 0 | 0 | 2150 | us | PWM extra 1 failsafe value |
| PWM_EXTRA_FAIL2 | Int32 | 0 | 0 | 2150 | us | PWM extra 2 failsafe value |
| PWM_EXTRA_FAIL3 | Int32 | 0 | 0 | 2150 | us | PWM extra 3 failsafe value |
| PWM_EXTRA_FAIL4 | Int32 | 0 | 0 | 2150 | us | PWM extra 4 failsafe value |
| PWM_EXTRA_FAIL5 | Int32 | 0 | 0 | 2150 | us | PWM extra 5 failsafe value |
| PWM_EXTRA_FAIL6 | Int32 | 0 | 0 | 2150 | us | PWM extra 6 failsafe value |
| PWM_EXTRA_FAIL7 | Int32 | 0 | 0 | 2150 | us | PWM extra 7 failsafe value |
| PWM_EXTRA_FAIL8 | Int32 | 0 | 0 | 2150 | us | PWM extra 8 failsafe value |
| PWM_EXTRA_MAX | Int32 | 2000 | 1600 | 2200 | us | PWM extra maximum value |
| PWM_EXTRA_MAX1 | Int32 | -1 | -1 | 2150 | us | PWM extra 1 maximum value |
| PWM_EXTRA_MAX2 | Int32 | -1 | -1 | 2150 | us | PWM extra 2 maximum value |
| PWM_EXTRA_MAX3 | Int32 | -1 | -1 | 2150 | us | PWM extra 3 maximum value |
| PWM_EXTRA_MAX4 | Int32 | -1 | -1 | 2150 | us | PWM extra 4 maximum value |
| PWM_EXTRA_MAX5 | Int32 | -1 | -1 | 2150 | us | PWM extra 5 maximum value |
| PWM_EXTRA_MAX6 | Int32 | -1 | -1 | 2150 | us | PWM extra 6 maximum value |
| PWM_EXTRA_MAX7 | Int32 | -1 | -1 | 2150 | us | PWM extra 7 maximum value |
| PWM_EXTRA_MAX8 | Int32 | -1 | -1 | 2150 | us | PWM extra 8 maximum value |
| PWM_EXTRA_MIN | Int32 | 1000 | 800 | 1400 | us | PWM extra minimum value |
| PWM_EXTRA_MIN1 | Int32 | -1 | -1 | 1600 | us | PWM extra 1 minimum value |
| PWM_EXTRA_MIN2 | Int32 | -1 | -1 | 1600 | us | PWM extra 2 minimum value |
| PWM_EXTRA_MIN3 | Int32 | -1 | -1 | 1600 | us | PWM extra 3 minimum value |
| PWM_EXTRA_MIN4 | Int32 | -1 | -1 | 1600 | us | PWM extra 4 minimum value |
| PWM_EXTRA_MIN5 | Int32 | -1 | -1 | 1600 | us | PWM extra 5 minimum value |
| PWM_EXTRA_MIN6 | Int32 | -1 | -1 | 1600 | us | PWM extra 6 minimum value |
| PWM_EXTRA_MIN7 | Int32 | -1 | -1 | 1600 | us | PWM extra 7 minimum value |
| PWM_EXTRA_MIN8 | Int32 | -1 | -1 | 1600 | us | PWM extra 8 minimum value |
| PWM_EXTRA_RATE | Int32 | 50 | -1 | 2000 | Hz | PWM extra output frequency |
| PWM_EXTRA_RATE1 | Int32 | 50 | 0 | 400 | Hz | PWM extra 1 rate |
| PWM_EXTRA_REV1 | Int32 | 0 | - | - | - | PWM extra 1 reverse value |
| PWM_EXTRA_REV2 | Int32 | 0 | - | - | - | PWM extra 2 reverse value |
| PWM_EXTRA_REV3 | Int32 | 0 | - | - | - | PWM extra 3 reverse value |
| PWM_EXTRA_REV4 | Int32 | 0 | - | - | - | PWM extra 4 reverse value |
| PWM_EXTRA_REV5 | Int32 | 0 | - | - | - | PWM extra 5 reverse value |
| PWM_EXTRA_REV6 | Int32 | 0 | - | - | - | PWM extra 6 reverse value |
| PWM_EXTRA_REV7 | Int32 | 0 | - | - | - | PWM extra 7 reverse value |
| PWM_EXTRA_REV8 | Int32 | 0 | - | - | - | PWM extra 8 reverse value |
| PWM_EXTRA_TRIM1 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 1 trim value |
| PWM_EXTRA_TRIM2 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 2 trim value |
| PWM_EXTRA_TRIM3 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 3 trim value |
| PWM_EXTRA_TRIM4 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 4 trim value |
| PWM_EXTRA_TRIM5 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 5 trim value |
| PWM_EXTRA_TRIM6 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 6 trim value |
| PWM_EXTRA_TRIM7 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 7 trim value |
| PWM_EXTRA_TRIM8 | Float | 0.0 | -0.2 | 0.2 | - | PWM extra 8 trim value |
| PWM_MAIN_DIS1 | Int32 | -1 | -1 | 2150 | us | PWM main 1 disarmed value |
| PWM_MAIN_DIS10 | Int32 | -1 | -1 | 2150 | us | PWM main 10 disarmed value |
| PWM_MAIN_DIS11 | Int32 | -1 | -1 | 2150 | us | PWM main 11 disarmed value |
| PWM_MAIN_DIS12 | Int32 | -1 | -1 | 2150 | us | PWM main 12 disarmed value |
| PWM_MAIN_DIS13 | Int32 | -1 | -1 | 2150 | us | PWM main 13 disarmed value |
| PWM_MAIN_DIS14 | Int32 | -1 | -1 | 2150 | us | PWM main 14 disarmed value |
| PWM_MAIN_DIS2 | Int32 | -1 | -1 | 2150 | us | PWM main 2 disarmed value |
| PWM_MAIN_DIS3 | Int32 | -1 | -1 | 2150 | us | PWM main 3 disarmed value |
| PWM_MAIN_DIS4 | Int32 | -1 | -1 | 2150 | us | PWM main 4 disarmed value |
| PWM_MAIN_DIS5 | Int32 | -1 | -1 | 2150 | us | PWM main 5 disarmed value |
| PWM_MAIN_DIS6 | Int32 | -1 | -1 | 2150 | us | PWM main 6 disarmed value |
| PWM_MAIN_DIS7 | Int32 | -1 | -1 | 2150 | us | PWM main 7 disarmed value |
| PWM_MAIN_DIS8 | Int32 | -1 | -1 | 2150 | us | PWM main 8 disarmed value |
| PWM_MAIN_DIS9 | Int32 | -1 | -1 | 2150 | us | PWM main 9 disarmed value |
| PWM_MAIN_DISARM | Int32 | 900 | 0 | 2200 | us | PWM main disarmed value |
| PWM_MAIN_FAIL1 | Int32 | -1 | -1 | 2150 | us | PWM main 1 failsafe value |
| PWM_MAIN_FAIL10 | Int32 | -1 | -1 | 2150 | us | PWM main 10 failsafe value |
| PWM_MAIN_FAIL11 | Int32 | -1 | -1 | 2150 | us | PWM main 11 failsafe value |
| PWM_MAIN_FAIL12 | Int32 | -1 | -1 | 2150 | us | PWM main 12 failsafe value |
| PWM_MAIN_FAIL13 | Int32 | -1 | -1 | 2150 | us | PWM main 13 failsafe value |
| PWM_MAIN_FAIL14 | Int32 | -1 | -1 | 2150 | us | PWM main 14 failsafe value |
| PWM_MAIN_FAIL2 | Int32 | -1 | -1 | 2150 | us | PWM main 2 failsafe value |
| PWM_MAIN_FAIL3 | Int32 | -1 | -1 | 2150 | us | PWM main 3 failsafe value |
| PWM_MAIN_FAIL4 | Int32 | -1 | -1 | 2150 | us | PWM main 4 failsafe value |
| PWM_MAIN_FAIL5 | Int32 | -1 | -1 | 2150 | us | PWM main 5 failsafe value |
| PWM_MAIN_FAIL6 | Int32 | -1 | -1 | 2150 | us | PWM main 6 failsafe value |
| PWM_MAIN_FAIL7 | Int32 | -1 | -1 | 2150 | us | PWM main 7 failsafe value |
| PWM_MAIN_FAIL8 | Int32 | -1 | -1 | 2150 | us | PWM main 8 failsafe value |
| PWM_MAIN_FAIL9 | Int32 | -1 | -1 | 2150 | us | PWM main 9 failsafe value |
| PWM_MAIN_MAX | Int32 | 2000 | 1600 | 2200 | us | PWM main maximum value |
| PWM_MAIN_MAX1 | Int32 | -1 | -1 | 2150 | us | PWM main 1 maximum value |
| PWM_MAIN_MAX10 | Int32 | -1 | -1 | 2150 | us | PWM main 10 maximum value |
| PWM_MAIN_MAX11 | Int32 | -1 | -1 | 2150 | us | PWM main 11 maximum value |
| PWM_MAIN_MAX12 | Int32 | -1 | -1 | 2150 | us | PWM main 12 maximum value |
| PWM_MAIN_MAX13 | Int32 | -1 | -1 | 2150 | us | PWM main 13 maximum value |
| PWM_MAIN_MAX14 | Int32 | -1 | -1 | 2150 | us | PWM main 14 maximum value |
| PWM_MAIN_MAX2 | Int32 | -1 | -1 | 2150 | us | PWM main 2 maximum value |
| PWM_MAIN_MAX3 | Int32 | -1 | -1 | 2150 | us | PWM main 3 maximum value |
| PWM_MAIN_MAX4 | Int32 | -1 | -1 | 2150 | us | PWM main 4 maximum value |
| PWM_MAIN_MAX5 | Int32 | -1 | -1 | 2150 | us | PWM main 5 maximum value |
| PWM_MAIN_MAX6 | Int32 | -1 | -1 | 2150 | us | PWM main 6 maximum value |
| PWM_MAIN_MAX7 | Int32 | -1 | -1 | 2150 | us | PWM main 7 maximum value |
| PWM_MAIN_MAX8 | Int32 | -1 | -1 | 2150 | us | PWM main 8 maximum value |
| PWM_MAIN_MAX9 | Int32 | -1 | -1 | 2150 | us | PWM main 9 maximum value |
| PWM_MAIN_MIN | Int32 | 1000 | 800 | 1400 | us | PWM main minimum value |
| PWM_MAIN_MIN1 | Int32 | -1 | -1 | 1600 | us | PWM main 1 minimum value |
| PWM_MAIN_MIN10 | Int32 | -1 | -1 | 1600 | us | PWM main 10 minimum value |
| PWM_MAIN_MIN11 | Int32 | -1 | -1 | 1600 | us | PWM main 11 minimum value |
| PWM_MAIN_MIN12 | Int32 | -1 | -1 | 1600 | us | PWM main 12 minimum value |
| PWM_MAIN_MIN13 | Int32 | -1 | -1 | 1600 | us | PWM main 13 minimum value |
| PWM_MAIN_MIN14 | Int32 | -1 | -1 | 1600 | us | PWM main 14 minimum value |
| PWM_MAIN_MIN2 | Int32 | -1 | -1 | 1600 | us | PWM main 2 minimum value |
| PWM_MAIN_MIN3 | Int32 | -1 | -1 | 1600 | us | PWM main 3 minimum value |
| PWM_MAIN_MIN4 | Int32 | -1 | -1 | 1600 | us | PWM main 4 minimum value |
| PWM_MAIN_MIN5 | Int32 | -1 | -1 | 1600 | us | PWM main 5 minimum value |
| PWM_MAIN_MIN6 | Int32 | -1 | -1 | 1600 | us | PWM main 6 minimum value |
| PWM_MAIN_MIN7 | Int32 | -1 | -1 | 1600 | us | PWM main 7 minimum value |
| PWM_MAIN_MIN8 | Int32 | -1 | -1 | 1600 | us | PWM main 8 minimum value |
| PWM_MAIN_MIN9 | Int32 | -1 | -1 | 1600 | us | PWM main 9 minimum value |
| PWM_MAIN_RATE | Int32 | 400 | -1 | 2000 | Hz | PWM main output frequency |
| PWM_MAIN_RATE1 | Int32 | 50 | 0 | 400 | Hz | PWM main 1 rate |
| PWM_MAIN_REV1 | Int32 | 0 | - | - | - | PWM main 1 reverse value |
| PWM_MAIN_REV10 | Int32 | 0 | - | - | - | PWM main 10 reverse value |
| PWM_MAIN_REV11 | Int32 | 0 | - | - | - | PWM main 11 reverse value |
| PWM_MAIN_REV12 | Int32 | 0 | - | - | - | PWM main 12 reverse value |
| PWM_MAIN_REV13 | Int32 | 0 | - | - | - | PWM main 13 reverse value |
| PWM_MAIN_REV14 | Int32 | 0 | - | - | - | PWM main 14 reverse value |
| PWM_MAIN_REV2 | Int32 | 0 | - | - | - | PWM main 2 reverse value |
| PWM_MAIN_REV3 | Int32 | 0 | - | - | - | PWM main 3 reverse value |
| PWM_MAIN_REV4 | Int32 | 0 | - | - | - | PWM main 4 reverse value |
| PWM_MAIN_REV5 | Int32 | 0 | - | - | - | PWM main 5 reverse value |
| PWM_MAIN_REV6 | Int32 | 0 | - | - | - | PWM main 6 reverse value |
| PWM_MAIN_REV7 | Int32 | 0 | - | - | - | PWM main 7 reverse value |
| PWM_MAIN_REV8 | Int32 | 0 | - | - | - | PWM main 8 reverse value |
| PWM_MAIN_REV9 | Int32 | 0 | - | - | - | PWM main 9 reverse value |
| PWM_MAIN_TRIM1 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 1 trim value |
| PWM_MAIN_TRIM10 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 10 trim value |
| PWM_MAIN_TRIM11 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 11 trim value |
| PWM_MAIN_TRIM12 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 12 trim value |
| PWM_MAIN_TRIM13 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 13 trim value |
| PWM_MAIN_TRIM14 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 14 trim value |
| PWM_MAIN_TRIM2 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 2 trim value |
| PWM_MAIN_TRIM3 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 3 trim value |
| PWM_MAIN_TRIM4 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 4 trim value |
| PWM_MAIN_TRIM5 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 5 trim value |
| PWM_MAIN_TRIM6 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 6 trim value |
| PWM_MAIN_TRIM7 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 7 trim value |
| PWM_MAIN_TRIM8 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 8 trim value |
| PWM_MAIN_TRIM9 | Float | 0.0 | -0.2 | 0.2 | - | PWM main 9 trim value |
| THR_MDL_FAC | Float | 0.0 | 0.0 | 1.0 | - | Thrust to motor control signal model parameter |

## Precision Land

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| PLD_BTOUT | Float | 5.0 | 0.0 | 50.0 | s | Landing Target Timeout |
| PLD_FAPPR_ALT | Float | 0.1 | 0.0 | 10.0 | m | Final approach altitude |
| PLD_HACC_RAD | Float | 0.2 | 0.0 | 10.0 | m | Horizontal acceptance radius |
| PLD_MAX_SRCH | Int32 | 3 | 0 | 100 | - | Maximum number of search attempts |
| PLD_SRCH_ALT | Float | 10.0 | 0.0 | 100.0 | m | Search altitude |
| PLD_SRCH_TOUT | Float | 10.0 | 0.0 | 100.0 | s | Search timeout |

## Radio Calibration

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| RC10_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 10 dead zone |
| RC10_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 10 maximum |
| RC10_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 10 minimum |
| RC10_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 10 reverse |
| RC10_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 10 trim |
| RC11_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 11 dead zone |
| RC11_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 11 maximum |
| RC11_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 11 minimum |
| RC11_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 11 reverse |
| RC11_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 11 trim |
| RC12_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 12 dead zone |
| RC12_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 12 maximum |
| RC12_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 12 minimum |
| RC12_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 12 reverse |
| RC12_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 12 trim |
| RC13_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 13 dead zone |
| RC13_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 13 maximum |
| RC13_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 13 minimum |
| RC13_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 13 reverse |
| RC13_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 13 trim |
| RC14_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 14 dead zone |
| RC14_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 14 maximum |
| RC14_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 14 minimum |
| RC14_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 14 reverse |
| RC14_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 14 trim |
| RC15_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 15 dead zone |
| RC15_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 15 maximum |
| RC15_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 15 minimum |
| RC15_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 15 reverse |
| RC15_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 15 trim |
| RC16_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 16 dead zone |
| RC16_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 16 maximum |
| RC16_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 16 minimum |
| RC16_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 16 reverse |
| RC16_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 16 trim |
| RC17_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 17 dead zone |
| RC17_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 17 maximum |
| RC17_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 17 minimum |
| RC17_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 17 reverse |
| RC17_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 17 trim |
| RC18_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 18 dead zone |
| RC18_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 18 maximum |
| RC18_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 18 minimum |
| RC18_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 18 reverse |
| RC18_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 18 trim |
| RC1_DZ | Float | 10.0 | 0.0 | 100.0 | us | RC channel 1 dead zone |
| RC1_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 1 maximum |
| RC1_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 1 minimum |
| RC1_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 1 reverse |
| RC1_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 1 trim |
| RC2_DZ | Float | 10.0 | 0.0 | 100.0 | us | RC channel 2 dead zone |
| RC2_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 2 maximum |
| RC2_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 2 minimum |
| RC2_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 2 reverse |
| RC2_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 2 trim |
| RC3_DZ | Float | 10.0 | 0.0 | 100.0 | us | RC channel 3 dead zone |
| RC3_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 3 maximum |
| RC3_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 3 minimum |
| RC3_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 3 reverse |
| RC3_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 3 trim |
| RC4_DZ | Float | 10.0 | 0.0 | 100.0 | us | RC channel 4 dead zone |
| RC4_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 4 maximum |
| RC4_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 4 minimum |
| RC4_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 4 reverse |
| RC4_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 4 trim |
| RC5_DZ | Float | 10.0 | 0.0 | 100.0 | - | RC channel 5 dead zone |
| RC5_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 5 maximum |
| RC5_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 5 minimum |
| RC5_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 5 reverse |
| RC5_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 5 trim |
| RC6_DZ | Float | 10.0 | 0.0 | 100.0 | - | RC channel 6 dead zone |
| RC6_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 6 maximum |
| RC6_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 6 minimum |
| RC6_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 6 reverse |
| RC6_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 6 trim |
| RC7_DZ | Float | 10.0 | 0.0 | 100.0 | - | RC channel 7 dead zone |
| RC7_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 7 maximum |
| RC7_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 7 minimum |
| RC7_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 7 reverse |
| RC7_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 7 trim |
| RC8_DZ | Float | 10.0 | 0.0 | 100.0 | - | RC channel 8 dead zone |
| RC8_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 8 maximum |
| RC8_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 8 minimum |
| RC8_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 8 reverse |
| RC8_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 8 trim |
| RC9_DZ | Float | 0.0 | 0.0 | 100.0 | - | RC channel 9 dead zone |
| RC9_MAX | Float | 2000.0 | 1500.0 | 2200.0 | us | RC channel 9 maximum |
| RC9_MIN | Float | 1000.0 | 800.0 | 1500.0 | us | RC channel 9 minimum |
| RC9_REV | Float | 1.0 | -1.0 | 1.0 | - | RC channel 9 reverse |
| RC9_TRIM | Float | 1500.0 | 800.0 | 2200.0 | us | RC channel 9 trim |
| RC_CHAN_CNT | Int32 | 0 | 0 | 18 | - | RC channel count |
| RC_FAILS_THR | Int32 | 0 | 0 | 2200 | us | Failsafe channel PWM threshold |
| RC_MAP_AUX1 | Int32 | 0 | 0 | 18 | - | AUX1 Passthrough RC channel |
| RC_MAP_AUX2 | Int32 | 0 | 0 | 18 | - | AUX2 Passthrough RC channel |
| RC_MAP_AUX3 | Int32 | 0 | 0 | 18 | - | AUX3 Passthrough RC channel |
| RC_MAP_AUX4 | Int32 | 0 | 0 | 18 | - | AUX4 Passthrough RC channel |
| RC_MAP_AUX5 | Int32 | 0 | 0 | 18 | - | AUX5 Passthrough RC channel |
| RC_MAP_AUX6 | Int32 | 0 | 0 | 18 | - | AUX6 Passthrough RC channel |
| RC_MAP_FAILSAFE | Int32 | 0 | 0 | 18 | - | Failsafe channel mapping |
| RC_MAP_PARAM1 | Int32 | 0 | 0 | 18 | - | PARAM1 tuning channel |
| RC_MAP_PARAM2 | Int32 | 0 | 0 | 18 | - | PARAM2 tuning channel |
| RC_MAP_PARAM3 | Int32 | 0 | 0 | 18 | - | PARAM3 tuning channel |
| RC_MAP_PITCH | Int32 | 0 | 0 | 18 | - | Pitch control channel mapping |
| RC_MAP_ROLL | Int32 | 0 | 0 | 18 | - | Roll control channel mapping |
| RC_MAP_THROTTLE | Int32 | 0 | 0 | 18 | - | Throttle control channel mapping |
| RC_MAP_YAW | Int32 | 0 | 0 | 18 | - | Yaw control channel mapping |
| RC_RSSI_PWM_CHAN | Int32 | 0 | 0 | 18 | - | PWM input channel that provides RSSI |
| RC_RSSI_PWM_MAX | Int32 | 2000 | 0 | 2000 | - | Max input value for RSSI reading |
| RC_RSSI_PWM_MIN | Int32 | 1000 | 0 | 2000 | - | Min input value for RSSI reading |
| TRIM_PITCH | Float | 0.0 | -0.25 | 0.25 | - | Pitch trim |
| TRIM_ROLL | Float | 0.0 | -0.25 | 0.25 | - | Roll trim |
| TRIM_YAW | Float | 0.0 | -0.25 | 0.25 | - | Yaw trim |

## Radio Switches

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| RC_ACRO_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting acro mode |
| RC_ARMSWITCH_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for the arm switch |
| RC_ASSIST_TH | Float | 0.25 | -1.0 | 1.0 | - | Threshold for selecting assist mode |
| RC_AUTO_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting auto mode |
| RC_GEAR_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for the landing gear switch |
| RC_KILLSWITCH_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for the kill switch |
| RC_LOITER_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting loiter mode |
| RC_MAN_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for the manual switch |
| RC_MAP_ACRO_SW | Int32 | 0 | 0 | 18 | - | Acro switch channel |
| RC_MAP_ARM_SW | Int32 | 0 | 0 | 18 | - | Arm switch channel |
| RC_MAP_FLAPS | Int32 | 0 | 0 | 18 | - | Flaps channel |
| RC_MAP_FLTMODE | Int32 | 0 | 0 | 18 | - | Single channel flight mode selection |
| RC_MAP_GEAR_SW | Int32 | 0 | 0 | 18 | - | Landing gear switch channel |
| RC_MAP_KILL_SW | Int32 | 0 | 0 | 18 | - | Emergency Kill switch channel |
| RC_MAP_LOITER_SW | Int32 | 0 | 0 | 18 | - | Loiter switch channel |
| RC_MAP_MAN_SW | Int32 | 0 | 0 | 18 | - | Manual switch channel mapping |
| RC_MAP_MODE_SW | Int32 | 0 | 0 | 18 | - | Mode switch channel mapping |
| RC_MAP_OFFB_SW | Int32 | 0 | 0 | 18 | - | Offboard switch channel |
| RC_MAP_POSCTL_SW | Int32 | 0 | 0 | 18 | - | Position Control switch channel |
| RC_MAP_RATT_SW | Int32 | 0 | 0 | 18 | - | Rattitude switch channel (deprecated) |
| RC_MAP_RETURN_SW | Int32 | 0 | 0 | 18 | - | Return switch channel |
| RC_MAP_STAB_SW | Int32 | 0 | 0 | 18 | - | Stabilize switch channel mapping |
| RC_MAP_TRANS_SW | Int32 | 0 | 0 | 18 | - | VTOL transition switch channel mapping |
| RC_OFFB_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting offboard mode |
| RC_POSCTL_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting posctl mode |
| RC_RETURN_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for selecting return to launch mode |
| RC_STAB_TH | Float | 0.5 | -1.0 | 1.0 | - | Threshold for the stabilize switch |
| RC_TRANS_TH | Float | 0.75 | -1.0 | 1.0 | - | Threshold for the VTOL transition switch |

## Return Mode

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| RTL_CONE_ANG | Int32 | 45 | 0 | 90 | deg | Half-angle of the return mode altitude cone |
| RTL_DESCEND_ALT | Float | 30.0 | 2.0 | 100.0 | m | Return mode loiter altitude |
| RTL_LAND_DELAY | Float | 0.0 | -1.0 | 300.0 | s | Return mode delay |
| RTL_LOITER_RAD | Float | 50.0 | 25.0 | 1000.0 | m | Loiter radius for rtl descend |
| RTL_MIN_DIST | Float | 10.0 | 0.5 | 100.0 | m | Horizontal radius from return point within which special rules for return mode apply |
| RTL_RETURN_ALT | Float | 60.0 | 0.0 | 150.0 | m | Return mode return altitude |
| RTL_TYPE | Int32 | 0 | - | - | - | Return type |

## Return To Land

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| RTL_PLD_MD | Int32 | 0 | - | - | - | RTL precision land mode |

## Rover Position Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| GND_L1_DAMPING | Float | 0.75 | 0.6 | 0.9 | - | L1 damping |
| GND_L1_DIST | Float | 1.0 | 1.0 | 50.0 | m | L1 distance |
| GND_L1_PERIOD | Float | 5.0 | 0.5 | 50.0 | m | L1 period |
| GND_MAN_Y_MAX | Float | 150.0 | 0.0 | 400.0 | deg/s | Max manual yaw rate |
| GND_MAX_ANG | Float | 0.7854 | 0.0 | 3.14159 | rad | Maximum turn angle for Ackerman steering |
| GND_SPEED_D | Float | 0.001 | 0.0 | 50.0 | %m/s | Speed proportional gain |
| GND_SPEED_I | Float | 3.0 | 0.0 | 50.0 | %m/s | Speed Integral gain |
| GND_SPEED_IMAX | Float | 1.0 | 0.005 | 50.0 | %m/s | Speed integral maximum value |
| GND_SPEED_MAX | Float | 10.0 | 0.0 | 40.0 | m/s | Maximum ground speed |
| GND_SPEED_P | Float | 2.0 | 0.005 | 50.0 | %m/s | Speed proportional gain |
| GND_SPEED_THR_SC | Float | 1.0 | 0.005 | 50.0 | %m/s | Speed to throttle scaler |
| GND_SPEED_TRIM | Float | 3.0 | 0.0 | 40.0 | m/s | Trim ground speed |
| GND_SP_CTRL_MODE | Int32 | 1 | 0 | 1 | - | Control mode for speed |
| GND_THR_CRUISE | Float | 0.1 | 0.0 | 1.0 | norm | Cruise throttle |
| GND_THR_MAX | Float | 0.3 | 0.0 | 1.0 | norm | Throttle limit max |
| GND_THR_MIN | Float | 0.0 | 0.0 | 1.0 | norm | Throttle limit min |
| GND_WHEEL_BASE | Float | 0.31 | 0.0 | - | m | Distance from front axle to rear axle |

## Runway Takeoff

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| RWTO_AIRSPD_SCL | Float | 1.3 | 0.0 | 2.0 | norm | Min airspeed scaling factor for takeoff |
| RWTO_HDG | Int32 | 0 | 0 | 1 | - | Specifies which heading should be held during runnway takeoff |
| RWTO_MAX_PITCH | Float | 20.0 | 0.0 | 60.0 | deg | Max pitch during takeoff |
| RWTO_MAX_ROLL | Float | 25.0 | 0.0 | 60.0 | deg | Max roll during climbout |
| RWTO_MAX_THR | Float | 1.0 | 0.0 | 1.0 | norm | Max throttle during runway takeoff |
| RWTO_NAV_ALT | Float | 5.0 | 0.0 | 100.0 | m | Altitude AGL at which we have enough ground clearance to allow some roll |
| RWTO_PSP | Float | 0.0 | -10.0 | 20.0 | deg | Pitch setpoint during taxi / before takeoff airspeed is reached |
| RWTO_RAMP_TIME | Float | 2.0 | 1.0 | 15.0 | s | Throttle ramp up time for runway takeoff |
| RWTO_TKOFF | Int32 | 0 | - | - | - | Runway takeoff with landing gear |

## SD Logging

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| SDLOG_BOOT_BAT | Int32 | 0 | - | - | - | Battery-only Logging |
| SDLOG_DIRS_MAX | Int32 | 0 | 0 | 1000 | - | Maximum number of log directories to keep |
| SDLOG_MISSION | Int32 | 0 | - | - | - | Mission Log |
| SDLOG_MODE | Int32 | 0 | - | - | - | Logging Mode |
| SDLOG_PROFILE | Int32 | 1 | 0 | 1023 | - | Logging topic profile (integer bitmask) |
| SDLOG_UTC_OFFSET | Int32 | 0 | -1000 | 1000 | min | UTC offset (unit: min) |
| SDLOG_UUID | Int32 | 1 | - | - | - | Log UUID |

## SITL

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| SIM_BAT_DRAIN | Float | 60.0 | 1.0 | 86400.0 | s | Simulator Battery drain interval |
| SIM_BAT_MIN_PCT | Float | 50.0 | 0.0 | 100.0 | % | Simulator Battery minimal percentage |

## Sensor Calibration

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CAL_ACC0_ID | Int32 | 0 | - | - | - | ID of the Accelerometer that the calibration is for |
| CAL_ACC0_PRIO | Int32 | -1 | - | - | - | Accelerometer 0 priority |
| CAL_ACC0_ROT | Int32 | -1 | -1 | 40 | - | Rotation of accelerometer 0 relative to airframe |
| CAL_ACC0_XOFF | Float | 0.0 | - | - | - | Accelerometer X-axis offset |
| CAL_ACC0_XSCALE | Float | 1.0 | - | - | - | Accelerometer X-axis scaling factor |
| CAL_ACC0_YOFF | Float | 0.0 | - | - | - | Accelerometer Y-axis offset |
| CAL_ACC0_YSCALE | Float | 1.0 | - | - | - | Accelerometer Y-axis scaling factor |
| CAL_ACC0_ZOFF | Float | 0.0 | - | - | - | Accelerometer Z-axis offset |
| CAL_ACC0_ZSCALE | Float | 1.0 | - | - | - | Accelerometer Z-axis scaling factor |
| CAL_ACC1_ID | Int32 | 0 | - | - | - | ID of the Accelerometer that the calibration is for |
| CAL_ACC1_PRIO | Int32 | -1 | - | - | - | Accelerometer 1 priority |
| CAL_ACC1_ROT | Int32 | -1 | -1 | 40 | - | Rotation of accelerometer 1 relative to airframe |
| CAL_ACC1_XOFF | Float | 0.0 | - | - | - | Accelerometer X-axis offset |
| CAL_ACC1_XSCALE | Float | 1.0 | - | - | - | Accelerometer X-axis scaling factor |
| CAL_ACC1_YOFF | Float | 0.0 | - | - | - | Accelerometer Y-axis offset |
| CAL_ACC1_YSCALE | Float | 1.0 | - | - | - | Accelerometer Y-axis scaling factor |
| CAL_ACC1_ZOFF | Float | 0.0 | - | - | - | Accelerometer Z-axis offset |
| CAL_ACC1_ZSCALE | Float | 1.0 | - | - | - | Accelerometer Z-axis scaling factor |
| CAL_ACC2_ID | Int32 | 0 | - | - | - | ID of the Accelerometer that the calibration is for |
| CAL_ACC2_PRIO | Int32 | -1 | - | - | - | Accelerometer 2 priority |
| CAL_ACC2_ROT | Int32 | -1 | -1 | 40 | - | Rotation of accelerometer 2 relative to airframe |
| CAL_ACC2_XOFF | Float | 0.0 | - | - | - | Accelerometer X-axis offset |
| CAL_ACC2_XSCALE | Float | 1.0 | - | - | - | Accelerometer X-axis scaling factor |
| CAL_ACC2_YOFF | Float | 0.0 | - | - | - | Accelerometer Y-axis offset |
| CAL_ACC2_YSCALE | Float | 1.0 | - | - | - | Accelerometer Y-axis scaling factor |
| CAL_ACC2_ZOFF | Float | 0.0 | - | - | - | Accelerometer Z-axis offset |
| CAL_ACC2_ZSCALE | Float | 1.0 | - | - | - | Accelerometer Z-axis scaling factor |
| CAL_ACC3_ID | Int32 | 0 | - | - | - | ID of the Accelerometer that the calibration is for |
| CAL_ACC3_PRIO | Int32 | -1 | - | - | - | Accelerometer 3 priority |
| CAL_ACC3_ROT | Int32 | -1 | -1 | 40 | - | Rotation of accelerometer 3 relative to airframe |
| CAL_ACC3_XOFF | Float | 0.0 | - | - | - | Accelerometer X-axis offset |
| CAL_ACC3_XSCALE | Float | 1.0 | - | - | - | Accelerometer X-axis scaling factor |
| CAL_ACC3_YOFF | Float | 0.0 | - | - | - | Accelerometer Y-axis offset |
| CAL_ACC3_YSCALE | Float | 1.0 | - | - | - | Accelerometer Y-axis scaling factor |
| CAL_ACC3_ZOFF | Float | 0.0 | - | - | - | Accelerometer Z-axis offset |
| CAL_ACC3_ZSCALE | Float | 1.0 | - | - | - | Accelerometer Z-axis scaling factor |
| CAL_GYRO0_ID | Int32 | 0 | - | - | - | ID of the Gyro that the calibration is for |
| CAL_GYRO0_PRIO | Int32 | -1 | - | - | - | Gyro 0 priority |
| CAL_GYRO0_ROT | Int32 | -1 | -1 | 40 | - | Rotation of gyro 0 relative to airframe |
| CAL_GYRO0_XOFF | Float | 0.0 | - | - | - | Gyro X-axis offset |
| CAL_GYRO0_YOFF | Float | 0.0 | - | - | - | Gyro Y-axis offset |
| CAL_GYRO0_ZOFF | Float | 0.0 | - | - | - | Gyro Z-axis offset |
| CAL_GYRO1_ID | Int32 | 0 | - | - | - | ID of the Gyro that the calibration is for |
| CAL_GYRO1_PRIO | Int32 | -1 | - | - | - | Gyro 1 priority |
| CAL_GYRO1_ROT | Int32 | -1 | -1 | 40 | - | Rotation of gyro 1 relative to airframe |
| CAL_GYRO1_XOFF | Float | 0.0 | - | - | - | Gyro X-axis offset |
| CAL_GYRO1_YOFF | Float | 0.0 | - | - | - | Gyro Y-axis offset |
| CAL_GYRO1_ZOFF | Float | 0.0 | - | - | - | Gyro Z-axis offset |
| CAL_GYRO2_ID | Int32 | 0 | - | - | - | ID of the Gyro that the calibration is for |
| CAL_GYRO2_PRIO | Int32 | -1 | - | - | - | Gyro 2 priority |
| CAL_GYRO2_ROT | Int32 | -1 | -1 | 40 | - | Rotation of gyro 2 relative to airframe |
| CAL_GYRO2_XOFF | Float | 0.0 | - | - | - | Gyro X-axis offset |
| CAL_GYRO2_YOFF | Float | 0.0 | - | - | - | Gyro Y-axis offset |
| CAL_GYRO2_ZOFF | Float | 0.0 | - | - | - | Gyro Z-axis offset |
| CAL_GYRO3_ID | Int32 | 0 | - | - | - | ID of the Gyro that the calibration is for |
| CAL_GYRO3_PRIO | Int32 | -1 | - | - | - | Gyro 3 priority |
| CAL_GYRO3_ROT | Int32 | -1 | -1 | 40 | - | Rotation of gyro 3 relative to airframe |
| CAL_GYRO3_XOFF | Float | 0.0 | - | - | - | Gyro X-axis offset |
| CAL_GYRO3_YOFF | Float | 0.0 | - | - | - | Gyro Y-axis offset |
| CAL_GYRO3_ZOFF | Float | 0.0 | - | - | - | Gyro Z-axis offset |
| CAL_MAG0_ID | Int32 | 0 | - | - | - | ID of Magnetometer the calibration is for |
| CAL_MAG0_PRIO | Int32 | -1 | - | - | - | Mag 0 priority |
| CAL_MAG0_ROT | Int32 | -1 | -1 | 40 | - | Rotation of magnetometer 0 relative to airframe |
| CAL_MAG0_XCOMP | Float | 0.0 | - | - | - | X Axis throttle compensation for Mag 0 |
| CAL_MAG0_XODIAG | Float | 0.0 | - | - | - | Magnetometer X-axis off diagonal factor |
| CAL_MAG0_XOFF | Float | 0.0 | - | - | - | Magnetometer X-axis offset |
| CAL_MAG0_XSCALE | Float | 1.0 | - | - | - | Magnetometer X-axis scaling factor |
| CAL_MAG0_YCOMP | Float | 0.0 | - | - | - | Y Axis throttle compensation for Mag 0 |
| CAL_MAG0_YODIAG | Float | 0.0 | - | - | - | Magnetometer Y-axis off diagonal factor |
| CAL_MAG0_YOFF | Float | 0.0 | - | - | - | Magnetometer Y-axis offset |
| CAL_MAG0_YSCALE | Float | 1.0 | - | - | - | Magnetometer Y-axis scaling factor |
| CAL_MAG0_ZCOMP | Float | 0.0 | - | - | - | Z Axis throttle compensation for Mag 0 |
| CAL_MAG0_ZODIAG | Float | 0.0 | - | - | - | Magnetometer Z-axis off diagonal factor |
| CAL_MAG0_ZOFF | Float | 0.0 | - | - | - | Magnetometer Z-axis offset |
| CAL_MAG0_ZSCALE | Float | 1.0 | - | - | - | Magnetometer Z-axis scaling factor |
| CAL_MAG1_ID | Int32 | 0 | - | - | - | ID of Magnetometer the calibration is for |
| CAL_MAG1_PRIO | Int32 | -1 | - | - | - | Mag 1 priority |
| CAL_MAG1_ROT | Int32 | -1 | -1 | 40 | - | Rotation of magnetometer 1 relative to airframe |
| CAL_MAG1_XCOMP | Float | 0.0 | - | - | - | X Axis throttle compensation for Mag 1 |
| CAL_MAG1_XODIAG | Float | 0.0 | - | - | - | Magnetometer X-axis off diagonal factor |
| CAL_MAG1_XOFF | Float | 0.0 | - | - | - | Magnetometer X-axis offset |
| CAL_MAG1_XSCALE | Float | 1.0 | - | - | - | Magnetometer X-axis scaling factor |
| CAL_MAG1_YCOMP | Float | 0.0 | - | - | - | Y Axis throttle compensation for Mag 1 |
| CAL_MAG1_YODIAG | Float | 0.0 | - | - | - | Magnetometer Y-axis off diagonal factor |
| CAL_MAG1_YOFF | Float | 0.0 | - | - | - | Magnetometer Y-axis offset |
| CAL_MAG1_YSCALE | Float | 1.0 | - | - | - | Magnetometer Y-axis scaling factor |
| CAL_MAG1_ZCOMP | Float | 0.0 | - | - | - | Z Axis throttle compensation for Mag 1 |
| CAL_MAG1_ZODIAG | Float | 0.0 | - | - | - | Magnetometer Z-axis off diagonal factor |
| CAL_MAG1_ZOFF | Float | 0.0 | - | - | - | Magnetometer Z-axis offset |
| CAL_MAG1_ZSCALE | Float | 1.0 | - | - | - | Magnetometer Z-axis scaling factor |
| CAL_MAG2_ID | Int32 | 0 | - | - | - | ID of Magnetometer the calibration is for |
| CAL_MAG2_PRIO | Int32 | -1 | - | - | - | Mag 2 priority |
| CAL_MAG2_ROT | Int32 | -1 | -1 | 40 | - | Rotation of magnetometer 2 relative to airframe |
| CAL_MAG2_XCOMP | Float | 0.0 | - | - | - | X Axis throttle compensation for Mag 2 |
| CAL_MAG2_XODIAG | Float | 0.0 | - | - | - | Magnetometer X-axis off diagonal factor |
| CAL_MAG2_XOFF | Float | 0.0 | - | - | - | Magnetometer X-axis offset |
| CAL_MAG2_XSCALE | Float | 1.0 | - | - | - | Magnetometer X-axis scaling factor |
| CAL_MAG2_YCOMP | Float | 0.0 | - | - | - | Y Axis throttle compensation for Mag 2 |
| CAL_MAG2_YODIAG | Float | 0.0 | - | - | - | Magnetometer Y-axis off diagonal factor |
| CAL_MAG2_YOFF | Float | 0.0 | - | - | - | Magnetometer Y-axis offset |
| CAL_MAG2_YSCALE | Float | 1.0 | - | - | - | Magnetometer Y-axis scaling factor |
| CAL_MAG2_ZCOMP | Float | 0.0 | - | - | - | Z Axis throttle compensation for Mag 2 |
| CAL_MAG2_ZODIAG | Float | 0.0 | - | - | - | Magnetometer Z-axis off diagonal factor |
| CAL_MAG2_ZOFF | Float | 0.0 | - | - | - | Magnetometer Z-axis offset |
| CAL_MAG2_ZSCALE | Float | 1.0 | - | - | - | Magnetometer Z-axis scaling factor |
| CAL_MAG3_ID | Int32 | 0 | - | - | - | ID of Magnetometer the calibration is for |
| CAL_MAG3_PRIO | Int32 | -1 | - | - | - | Mag 3 priority |
| CAL_MAG3_ROT | Int32 | -1 | -1 | 40 | - | Rotation of magnetometer 3 relative to airframe |
| CAL_MAG3_XCOMP | Float | 0.0 | - | - | - | X Axis throttle compensation for Mag 3 |
| CAL_MAG3_XODIAG | Float | 0.0 | - | - | - | Magnetometer X-axis off diagonal factor |
| CAL_MAG3_XOFF | Float | 0.0 | - | - | - | Magnetometer X-axis offset |
| CAL_MAG3_XSCALE | Float | 1.0 | - | - | - | Magnetometer X-axis scaling factor |
| CAL_MAG3_YCOMP | Float | 0.0 | - | - | - | Y Axis throttle compensation for Mag 3 |
| CAL_MAG3_YODIAG | Float | 0.0 | - | - | - | Magnetometer Y-axis off diagonal factor |
| CAL_MAG3_YOFF | Float | 0.0 | - | - | - | Magnetometer Y-axis offset |
| CAL_MAG3_YSCALE | Float | 1.0 | - | - | - | Magnetometer Y-axis scaling factor |
| CAL_MAG3_ZCOMP | Float | 0.0 | - | - | - | Z Axis throttle compensation for Mag 3 |
| CAL_MAG3_ZODIAG | Float | 0.0 | - | - | - | Magnetometer Z-axis off diagonal factor |
| CAL_MAG3_ZOFF | Float | 0.0 | - | - | - | Magnetometer Z-axis offset |
| CAL_MAG3_ZSCALE | Float | 1.0 | - | - | - | Magnetometer Z-axis scaling factor |
| CAL_MAG_COMP_TYP | Int32 | 0 | - | - | - | Type of magnetometer compensation |
| SENS_DPRES_ANSC | Float | 0.0 | - | - | - | Differential pressure sensor analog scaling |
| SENS_DPRES_OFF | Float | 0.0 | - | - | - | Differential pressure sensor offset |
| SENS_FLOW_MAXHGT | Float | 3.0 | 1.0 | 25.0 | m | Maximum height above ground when reliant on optical flow |
| SENS_FLOW_MAXR | Float | 2.5 | 1.0 | - | rad/s | Magnitude of maximum angular flow rate reliably measurable by the optical flow sensor |
| SENS_FLOW_MINHGT | Float | 0.7 | 0.0 | 1.0 | m | Minimum height above ground when reliant on optical flow |

## Sensors

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| CAL_AIR_CMODEL | Int32 | 0 | - | - | - | Airspeed sensor compensation model for the SDP3x |
| CAL_AIR_TUBED_MM | Float | 1.5 | 0.1 | 100.0 | mm | Airspeed sensor tube diameter. Only used for the Tube Pressure Drop Compensation |
| CAL_AIR_TUBELEN | Float | 0.2 | 0.01 | 2.0 | m | Airspeed sensor tube length |
| CAL_MAG_ROT_AUTO | Int32 | 1 | - | - | - | Automatically set external rotations |
| CAL_MAG_SIDES | Int32 | 63 | 34 | 63 | - | Bitfield selecting mag sides for calibration |
| IMU_ACCEL_CUTOFF | Float | 30.0 | 0.0 | 1000.0 | Hz | Low pass filter cutoff frequency for accel |
| IMU_DGYRO_CUTOFF | Float | 30.0 | 0.0 | 1000.0 | Hz | Cutoff frequency for angular acceleration (D-Term filter) |
| IMU_GYRO_CAL_EN | Int32 | 1 | - | - | - | IMU gyro auto calibration enable |
| IMU_GYRO_CUTOFF | Float | 40.0 | 0.0 | 1000.0 | Hz | Low pass filter cutoff frequency for gyro |
| IMU_GYRO_DYN_NF | Int32 | 0 | 0 | 3 | - | IMU gyro dynamic notch filtering |
| IMU_GYRO_FFT_EN | Int32 | 0 | - | - | - | IMU gyro FFT enable |
| IMU_GYRO_FFT_LEN | Int32 | 1024 | - | - | Hz | IMU gyro FFT length |
| IMU_GYRO_FFT_MAX | Float | 192.0 | 1.0 | 1000.0 | Hz | IMU gyro FFT maximum frequency |
| IMU_GYRO_FFT_MIN | Float | 32.0 | 1.0 | 1000.0 | Hz | IMU gyro FFT minimum frequency |
| IMU_GYRO_NF_BW | Float | 20.0 | 0.0 | 100.0 | Hz | Notch filter bandwidth for gyro |
| IMU_GYRO_NF_FREQ | Float | 0.0 | 0.0 | 1000.0 | Hz | Notch filter frequency for gyro |
| IMU_GYRO_RATEMAX | Int32 | 400 | 100 | 2000 | Hz | Gyro control data maximum publication rate (inner loop rate) |
| IMU_INTEG_RATE | Int32 | 200 | 100 | 1000 | Hz | IMU integration rate |
| SENS_BARO_QNH | Float | 1013.25 | 500.0 | 1500.0 | hPa | QNH for barometer |
| SENS_BARO_RATE | Float | 20.0 | 1.0 | 200.0 | Hz | Baro max rate |
| SENS_BOARD_ROT | Int32 | 0 | -1 | 40 | - | Board rotation |
| SENS_BOARD_X_OFF | Float | 0.0 | - | - | deg | Board rotation X (Roll) offset |
| SENS_BOARD_Y_OFF | Float | 0.0 | - | - | deg | Board rotation Y (Pitch) offset |
| SENS_BOARD_Z_OFF | Float | 0.0 | - | - | deg | Board rotation Z (YAW) offset |
| SENS_EN_THERMAL | Int32 | -1 | - | - | - | Thermal control of sensor temperature |
| SENS_EXT_I2C_PRB | Int32 | 1 | - | - | - | External I2C probe |
| SENS_FLOW_ROT | Int32 | 6 | - | - | - | PX4Flow board rotation |
| SENS_GPS_MASK | Int32 | 0 | 0 | 7 | - | Multi GPS Blending Control Mask |
| SENS_GPS_PRIME | Int32 | 0 | -1 | 1 | - | Multi GPS primary instance |
| SENS_GPS_TAU | Float | 10.0 | 1.0 | 100.0 | s | Multi GPS Blending Time Constant |
| SENS_IMU_MODE | Int32 | 1 | - | - | - | Sensors hub IMU mode |
| SENS_INT_BARO_EN | Int32 | 1 | - | - | - | Enable internal barometers |
| SENS_MAG_MODE | Int32 | 1 | - | - | - | Sensors hub mag mode |
| SENS_MAG_RATE | Float | 50.0 | 1.0 | 200.0 | Hz | Magnetometer max rate |

## System

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| SYS_AUTOCONFIG | Int32 | 0 | - | - | - | Automatically configure default values |
| SYS_AUTOSTART | Int32 | 0 | 0 | 9999999 | - | Auto-start script index |
| SYS_BL_UPDATE | Int32 | 0 | - | - | - | Bootloader update |
| SYS_CAL_ACCEL | Int32 | 0 | 0 | 1 | - | Enable auto start of accelerometer thermal calibration at the next power up |
| SYS_CAL_BARO | Int32 | 0 | 0 | 1 | - | Enable auto start of barometer thermal calibration at the next power up |
| SYS_CAL_GYRO | Int32 | 0 | 0 | 1 | - | Enable auto start of rate gyro thermal calibration at the next power up |
| SYS_CAL_TDEL | Int32 | 24 | 10 | - | celcius | Required temperature rise during thermal calibration |
| SYS_CAL_TMAX | Int32 | 10 | - | - | celcius | Maximum starting temperature for thermal calibration |
| SYS_CAL_TMIN | Int32 | 5 | - | - | celcius | Minimum starting temperature for thermal calibration |
| SYS_FAC_CAL_MODE | Int32 | 0 | - | - | - | Enable factory calibration mode |
| SYS_FAILURE_EN | Int32 | 0 | - | - | - | Enable failure injection |
| SYS_HAS_BARO | Int32 | 1 | - | - | - | Control if the vehicle has a barometer |
| SYS_HAS_MAG | Int32 | 1 | - | - | - | Control if the vehicle has a magnetometer |
| SYS_HITL | Int32 | 0 | - | - | - | Enable HITL/SIH mode on next boot |
| SYS_MC_EST_GROUP | Int32 | 2 | - | - | - | Set multicopter estimator group |
| SYS_RESTART_TYPE | Int32 | 2 | 0 | 2 | - | Set restart type |
| SYS_STCK_EN | Int32 | 1 | - | - | - | Enable stack checking |

## Testing

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| TEST_1 | Int32 | 2 | - | - | - | TEST_1 |
| TEST_2 | Int32 | 4 | - | - | - | TEST_2 |
| TEST_3 | Float | 5.0 | - | - | - | TEST_3 |
| TEST_D | Float | 0.01 | - | - | - | TEST_D |
| TEST_DEV | Float | 2.0 | - | - | - | TEST_DEV |
| TEST_D_LP | Float | 10.0 | - | - | - | TEST_D_LP |
| TEST_HP | Float | 10.0 | - | - | - | TEST_HP |
| TEST_I | Float | 0.1 | - | - | - | TEST_I |
| TEST_I_MAX | Float | 1.0 | - | - | - | TEST_I_MAX |
| TEST_LP | Float | 10.0 | - | - | - | TEST_LP |
| TEST_MAX | Float | 1.0 | - | - | - | TEST_MAX |
| TEST_MEAN | Float | 1.0 | - | - | - | TEST_MEAN |
| TEST_MIN | Float | -1.0 | - | - | - | TEST_MIN |
| TEST_P | Float | 0.2 | - | - | - | TEST_P |
| TEST_PARAMS | Int32 | 12345678 | - | - | - | TEST_PARAMS |
| TEST_RC2_X | Int32 | 16 | - | - | - | TEST_RC2_X |
| TEST_RC_X | Int32 | 8 | - | - | - | TEST_RC_X |
| TEST_TRIM | Float | 0.5 | - | - | - | TEST_TRIM |

## Thermal Compensation

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| TC_A0_ID | Int32 | 0 | - | - | - | ID of Accelerometer that the calibration is for |
| TC_A0_TMAX | Float | 100.0 | - | - | - | Accelerometer calibration maximum temperature |
| TC_A0_TMIN | Float | 0.0 | - | - | - | Accelerometer calibration minimum temperature |
| TC_A0_TREF | Float | 25.0 | - | - | - | Accelerometer calibration reference temperature |
| TC_A0_X0_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - X axis |
| TC_A0_X0_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Y axis |
| TC_A0_X0_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Z axis |
| TC_A0_X1_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - X axis |
| TC_A0_X1_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Y axis |
| TC_A0_X1_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Z axis |
| TC_A0_X2_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - X axis |
| TC_A0_X2_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Y axis |
| TC_A0_X2_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Z axis |
| TC_A0_X3_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - X axis |
| TC_A0_X3_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Y axis |
| TC_A0_X3_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Z axis |
| TC_A1_ID | Int32 | 0 | - | - | - | ID of Accelerometer that the calibration is for |
| TC_A1_TMAX | Float | 100.0 | - | - | - | Accelerometer calibration maximum temperature |
| TC_A1_TMIN | Float | 0.0 | - | - | - | Accelerometer calibration minimum temperature |
| TC_A1_TREF | Float | 25.0 | - | - | - | Accelerometer calibration reference temperature |
| TC_A1_X0_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - X axis |
| TC_A1_X0_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Y axis |
| TC_A1_X0_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Z axis |
| TC_A1_X1_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - X axis |
| TC_A1_X1_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Y axis |
| TC_A1_X1_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Z axis |
| TC_A1_X2_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - X axis |
| TC_A1_X2_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Y axis |
| TC_A1_X2_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Z axis |
| TC_A1_X3_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - X axis |
| TC_A1_X3_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Y axis |
| TC_A1_X3_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Z axis |
| TC_A2_ID | Int32 | 0 | - | - | - | ID of Accelerometer that the calibration is for |
| TC_A2_TMAX | Float | 100.0 | - | - | - | Accelerometer calibration maximum temperature |
| TC_A2_TMIN | Float | 0.0 | - | - | - | Accelerometer calibration minimum temperature |
| TC_A2_TREF | Float | 25.0 | - | - | - | Accelerometer calibration reference temperature |
| TC_A2_X0_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - X axis |
| TC_A2_X0_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Y axis |
| TC_A2_X0_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Z axis |
| TC_A2_X1_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - X axis |
| TC_A2_X1_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Y axis |
| TC_A2_X1_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Z axis |
| TC_A2_X2_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - X axis |
| TC_A2_X2_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Y axis |
| TC_A2_X2_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Z axis |
| TC_A2_X3_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - X axis |
| TC_A2_X3_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Y axis |
| TC_A2_X3_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Z axis |
| TC_A3_ID | Int32 | 0 | - | - | - | ID of Accelerometer that the calibration is for |
| TC_A3_TMAX | Float | 100.0 | - | - | - | Accelerometer calibration maximum temperature |
| TC_A3_TMIN | Float | 0.0 | - | - | - | Accelerometer calibration minimum temperature |
| TC_A3_TREF | Float | 25.0 | - | - | - | Accelerometer calibration reference temperature |
| TC_A3_X0_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - X axis |
| TC_A3_X0_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Y axis |
| TC_A3_X0_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^0 polynomial coefficient - Z axis |
| TC_A3_X1_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - X axis |
| TC_A3_X1_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Y axis |
| TC_A3_X1_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^1 polynomial coefficient - Z axis |
| TC_A3_X2_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - X axis |
| TC_A3_X2_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Y axis |
| TC_A3_X2_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^2 polynomial coefficient - Z axis |
| TC_A3_X3_0 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - X axis |
| TC_A3_X3_1 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Y axis |
| TC_A3_X3_2 | Float | 0.0 | - | - | - | Accelerometer offset temperature ^3 polynomial coefficient - Z axis |
| TC_A_ENABLE | Int32 | 0 | - | - | - | Thermal compensation for accelerometer sensors |
| TC_B0_ID | Int32 | 0 | - | - | - | ID of Barometer that the calibration is for |
| TC_B0_TMAX | Float | 75.0 | - | - | - | Barometer calibration maximum temperature |
| TC_B0_TMIN | Float | 5.0 | - | - | - | Barometer calibration minimum temperature |
| TC_B0_TREF | Float | 40.0 | - | - | - | Barometer calibration reference temperature |
| TC_B0_X0 | Float | 0.0 | - | - | - | Barometer offset temperature ^0 polynomial coefficient |
| TC_B0_X1 | Float | 0.0 | - | - | - | Barometer offset temperature ^1 polynomial coefficients |
| TC_B0_X2 | Float | 0.0 | - | - | - | Barometer offset temperature ^2 polynomial coefficient |
| TC_B0_X3 | Float | 0.0 | - | - | - | Barometer offset temperature ^3 polynomial coefficient |
| TC_B0_X4 | Float | 0.0 | - | - | - | Barometer offset temperature ^4 polynomial coefficient |
| TC_B0_X5 | Float | 0.0 | - | - | - | Barometer offset temperature ^5 polynomial coefficient |
| TC_B1_ID | Int32 | 0 | - | - | - | ID of Barometer that the calibration is for |
| TC_B1_TMAX | Float | 75.0 | - | - | - | Barometer calibration maximum temperature |
| TC_B1_TMIN | Float | 5.0 | - | - | - | Barometer calibration minimum temperature |
| TC_B1_TREF | Float | 40.0 | - | - | - | Barometer calibration reference temperature |
| TC_B1_X0 | Float | 0.0 | - | - | - | Barometer offset temperature ^0 polynomial coefficient |
| TC_B1_X1 | Float | 0.0 | - | - | - | Barometer offset temperature ^1 polynomial coefficients |
| TC_B1_X2 | Float | 0.0 | - | - | - | Barometer offset temperature ^2 polynomial coefficient |
| TC_B1_X3 | Float | 0.0 | - | - | - | Barometer offset temperature ^3 polynomial coefficient |
| TC_B1_X4 | Float | 0.0 | - | - | - | Barometer offset temperature ^4 polynomial coefficient |
| TC_B1_X5 | Float | 0.0 | - | - | - | Barometer offset temperature ^5 polynomial coefficient |
| TC_B2_ID | Int32 | 0 | - | - | - | ID of Barometer that the calibration is for |
| TC_B2_TMAX | Float | 75.0 | - | - | - | Barometer calibration maximum temperature |
| TC_B2_TMIN | Float | 5.0 | - | - | - | Barometer calibration minimum temperature |
| TC_B2_TREF | Float | 40.0 | - | - | - | Barometer calibration reference temperature |
| TC_B2_X0 | Float | 0.0 | - | - | - | Barometer offset temperature ^0 polynomial coefficient |
| TC_B2_X1 | Float | 0.0 | - | - | - | Barometer offset temperature ^1 polynomial coefficients |
| TC_B2_X2 | Float | 0.0 | - | - | - | Barometer offset temperature ^2 polynomial coefficient |
| TC_B2_X3 | Float | 0.0 | - | - | - | Barometer offset temperature ^3 polynomial coefficient |
| TC_B2_X4 | Float | 0.0 | - | - | - | Barometer offset temperature ^4 polynomial coefficient |
| TC_B2_X5 | Float | 0.0 | - | - | - | Barometer offset temperature ^5 polynomial coefficient |
| TC_B3_ID | Int32 | 0 | - | - | - | ID of Barometer that the calibration is for |
| TC_B3_TMAX | Float | 75.0 | - | - | - | Barometer calibration maximum temperature |
| TC_B3_TMIN | Float | 5.0 | - | - | - | Barometer calibration minimum temperature |
| TC_B3_TREF | Float | 40.0 | - | - | - | Barometer calibration reference temperature |
| TC_B3_X0 | Float | 0.0 | - | - | - | Barometer offset temperature ^0 polynomial coefficient |
| TC_B3_X1 | Float | 0.0 | - | - | - | Barometer offset temperature ^1 polynomial coefficients |
| TC_B3_X2 | Float | 0.0 | - | - | - | Barometer offset temperature ^2 polynomial coefficient |
| TC_B3_X3 | Float | 0.0 | - | - | - | Barometer offset temperature ^3 polynomial coefficient |
| TC_B3_X4 | Float | 0.0 | - | - | - | Barometer offset temperature ^4 polynomial coefficient |
| TC_B3_X5 | Float | 0.0 | - | - | - | Barometer offset temperature ^5 polynomial coefficient |
| TC_B_ENABLE | Int32 | 0 | - | - | - | Thermal compensation for barometric pressure sensors |
| TC_G0_ID | Int32 | 0 | - | - | - | ID of Gyro that the calibration is for |
| TC_G0_TMAX | Float | 100.0 | - | - | - | Gyro calibration maximum temperature |
| TC_G0_TMIN | Float | 0.0 | - | - | - | Gyro calibration minimum temperature |
| TC_G0_TREF | Float | 25.0 | - | - | - | Gyro calibration reference temperature |
| TC_G0_X0_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - X axis |
| TC_G0_X0_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Y axis |
| TC_G0_X0_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Z axis |
| TC_G0_X1_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - X axis |
| TC_G0_X1_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Y axis |
| TC_G0_X1_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Z axis |
| TC_G0_X2_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - X axis |
| TC_G0_X2_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Y axis |
| TC_G0_X2_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Z axis |
| TC_G0_X3_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - X axis |
| TC_G0_X3_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Y axis |
| TC_G0_X3_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Z axis |
| TC_G1_ID | Int32 | 0 | - | - | - | ID of Gyro that the calibration is for |
| TC_G1_TMAX | Float | 100.0 | - | - | - | Gyro calibration maximum temperature |
| TC_G1_TMIN | Float | 0.0 | - | - | - | Gyro calibration minimum temperature |
| TC_G1_TREF | Float | 25.0 | - | - | - | Gyro calibration reference temperature |
| TC_G1_X0_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - X axis |
| TC_G1_X0_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Y axis |
| TC_G1_X0_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Z axis |
| TC_G1_X1_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - X axis |
| TC_G1_X1_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Y axis |
| TC_G1_X1_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Z axis |
| TC_G1_X2_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - X axis |
| TC_G1_X2_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Y axis |
| TC_G1_X2_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Z axis |
| TC_G1_X3_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - X axis |
| TC_G1_X3_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Y axis |
| TC_G1_X3_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Z axis |
| TC_G2_ID | Int32 | 0 | - | - | - | ID of Gyro that the calibration is for |
| TC_G2_TMAX | Float | 100.0 | - | - | - | Gyro calibration maximum temperature |
| TC_G2_TMIN | Float | 0.0 | - | - | - | Gyro calibration minimum temperature |
| TC_G2_TREF | Float | 25.0 | - | - | - | Gyro calibration reference temperature |
| TC_G2_X0_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - X axis |
| TC_G2_X0_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Y axis |
| TC_G2_X0_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Z axis |
| TC_G2_X1_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - X axis |
| TC_G2_X1_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Y axis |
| TC_G2_X1_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Z axis |
| TC_G2_X2_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - X axis |
| TC_G2_X2_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Y axis |
| TC_G2_X2_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Z axis |
| TC_G2_X3_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - X axis |
| TC_G2_X3_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Y axis |
| TC_G2_X3_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Z axis |
| TC_G3_ID | Int32 | 0 | - | - | - | ID of Gyro that the calibration is for |
| TC_G3_TMAX | Float | 100.0 | - | - | - | Gyro calibration maximum temperature |
| TC_G3_TMIN | Float | 0.0 | - | - | - | Gyro calibration minimum temperature |
| TC_G3_TREF | Float | 25.0 | - | - | - | Gyro calibration reference temperature |
| TC_G3_X0_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - X axis |
| TC_G3_X0_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Y axis |
| TC_G3_X0_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^0 polynomial coefficient - Z axis |
| TC_G3_X1_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - X axis |
| TC_G3_X1_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Y axis |
| TC_G3_X1_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^1 polynomial coefficient - Z axis |
| TC_G3_X2_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - X axis |
| TC_G3_X2_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Y axis |
| TC_G3_X2_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^2 polynomial coefficient - Z axis |
| TC_G3_X3_0 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - X axis |
| TC_G3_X3_1 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Y axis |
| TC_G3_X3_2 | Float | 0.0 | - | - | - | Gyro rate offset temperature ^3 polynomial coefficient - Z axis |
| TC_G_ENABLE | Int32 | 0 | - | - | - | Thermal compensation for rate gyro sensors |

## UAVCAN GNSS

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| gnss.dyn_model | Int32 | 2 | 0 | 2 | - | GNSS dynamic model |
| gnss.old_fix_msg | Int32 | 1 | 0 | 1 | - | Broadcast old GNSS fix message |
| gnss.warn_dimens | Int32 | 0 | 0 | 3 | - | device health warning |
| gnss.warn_sats | Int32 | 0 | - | - | - | - |
| uavcan.pubp-pres | Int32 | 0 | 0 | 1000000 | us | - |

## UAVCAN Motor Parameters

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| ctl_bw | Int32 | 75 | 10 | 250 | Hz | Speed controller bandwidth |
| ctl_dir | Int32 | 1 | 0 | 1 | - | Reverse direction |
| ctl_gain | Float | 1.0 | 0.0 | 1.0 | C/rad | Speed (RPM) controller gain |
| ctl_hz_idle | Float | 3.5 | 0.0 | 100.0 | Hz | Idle speed (e Hz) |
| ctl_start_rate | Int32 | 25 | 5 | 1000 | 1/s^2 | Spin-up rate (e Hz/s) |
| esc_index | Int32 | 0 | 0 | 15 | - | Index of this ESC in throttle command messages. |
| id_ext_status | Int32 | 20034 | 1 | 1000000 | - | Extended status ID |
| int_ext_status | Int32 | 50000 | 0 | 1000000 | us | Extended status interval (µs) |
| int_status | Int32 | 50000 | - | 1000000 | us | ESC status interval (µs) |
| mot_i_max | Float | 12.0 | 1.0 | 80.0 | A | Motor current limit in amps |
| mot_kv | Int32 | 2300 | 0 | 4000 | rpm/V | Motor Kv in RPM per volt |
| mot_ls | Float | 0.0 | - | - | H | READ ONLY: Motor inductance in henries. |
| mot_num_poles | Int32 | 14 | 2 | 40 | - | Number of motor poles. |
| mot_rs | Float | 0.0 | - | - | Ohm | READ ONLY: Motor resistance in ohms |
| mot_v_accel | Float | 0.5 | 0.01 | 1.0 | V | Acceleration limit (V) |
| mot_v_max | Float | 14.8 | 0.0 | - | V | Motor voltage limit in volts |

## UUV Attitude Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| UUV_DIRCT_PITCH | Float | 0.0 | - | - | - | Direct pitch input |
| UUV_DIRCT_ROLL | Float | 0.0 | - | - | - | Direct roll input |
| UUV_DIRCT_THRUST | Float | 0.0 | - | - | - | Direct thrust input |
| UUV_DIRCT_YAW | Float | 0.0 | - | - | - | Direct yaw input |
| UUV_INPUT_MODE | Int32 | 0 | - | - | - | Select Input Mode |
| UUV_PITCH_D | Float | 2.0 | - | - | - | Pitch differential gain |
| UUV_PITCH_P | Float | 4.0 | - | - | - | Pitch proportional gain |
| UUV_ROLL_D | Float | 1.5 | - | - | - | Roll differential gain |
| UUV_ROLL_P | Float | 4.0 | - | - | - | Roll proportional gain |
| UUV_YAW_D | Float | 2.0 | - | - | - | Yaw differential gain |
| UUV_YAW_P | Float | 4.0 | - | - | - | Yawh proportional gain |

## UUV Position Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| UUV_GAIN_X_D | Float | 0.2 | - | - | - | Gain of D controller X |
| UUV_GAIN_X_P | Float | 1.0 | - | - | - | Gain of P controller X |
| UUV_GAIN_Y_D | Float | 0.2 | - | - | - | Gain of D controller Y |
| UUV_GAIN_Y_P | Float | 1.0 | - | - | - | Gain of P controller Y |
| UUV_GAIN_Z_D | Float | 0.2 | - | - | - | Gain of D controller Z |
| UUV_GAIN_Z_P | Float | 1.0 | - | - | - | Gain of P controller Z |
| UUV_STAB_MODE | Int32 | 1 | - | - | - | Stabilization mode(1) or Position Control(0) |

## VTOL Attitude Control

| Parameter | Type | Default | Min | Max | Unit | Description |
|-----------|------|---------|-----|-----|------|-------------|
| VT_ARSP_BLEND | Float | 8.0 | 0.0 | 30.0 | m/s | Transition blending airspeed |
| VT_ARSP_TRANS | Float | 10.0 | 0.0 | 30.0 | m/s | Transition airspeed |
| VT_B_DEC_FF | Float | 0.12 | 0.0 | 0.2 | rad s^2/m | Backtransition deceleration setpoint to pitch feedforward gain |
| VT_B_DEC_I | Float | 0.1 | 0.0 | 0.3 | rad s/m | Backtransition deceleration setpoint to pitch I gain |
| VT_B_DEC_MSS | Float | 2.0 | 0.5 | 10.0 | m/s^2 | Approximate deceleration during back transition |
| VT_B_REV_DEL | Float | 0.0 | 0.0 | 10.0 | - | Delay in seconds before applying back transition throttle |
| VT_B_REV_OUT | Float | 0.0 | 0.0 | 1.0 | - | Output on airbrakes channel during back transition |
| VT_B_TRANS_DUR | Float | 4.0 | 0.0 | 20.0 | s | Duration of a back transition |
| VT_B_TRANS_RAMP | Float | 3.0 | 0.0 | 20.0 | s | Back transition MC motor ramp up time |
| VT_B_TRANS_THR | Float | 0.0 | -1.0 | 1.0 | - | Target throttle value for the transition to hover flight |
| VT_DWN_PITCH_MAX | Float | 5.0 | 0.0 | 45.0 | - | Maximum allowed angle the vehicle is allowed to pitch down to generate forward force |
| VT_ELEV_MC_LOCK | Int32 | 1 | - | - | - | Lock elevons in multicopter mode |
| VT_FWD_THRUST_EN | Int32 | 0 | - | - | - | Enable/disable usage of fixed-wing actuators in hover to generate forward force (instead of pitching down) |
| VT_FWD_THRUST_SC | Float | 0.7 | 0.0 | 2.0 | - | Fixed-wing actuator thrust scale for hover forward flight |
| VT_FW_ALT_ERR | Float | 0.0 | 0.0 | 200.0 | - | Adaptive QuadChute |
| VT_FW_DIFTHR_EN | Int32 | 0 | 0 | 1 | - | Differential thrust in forwards flight |
| VT_FW_DIFTHR_SC | Float | 0.1 | 0.0 | 1.0 | - | Differential thrust scaling factor |
| VT_FW_MIN_ALT | Float | 0.0 | 0.0 | 200.0 | - | QuadChute Altitude |
| VT_FW_MOT_OFFID | Int32 | 0 | 0 | 12345678 | - | The channel number of motors that must be turned off in fixed wing mode |
| VT_FW_PERM_STAB | Int32 | 0 | - | - | - | Permanent stabilization in fw mode |
| VT_FW_QC_P | Int32 | 0 | 0 | 180 | - | QuadChute Max Pitch |
| VT_FW_QC_R | Int32 | 0 | 0 | 180 | - | QuadChute Max Roll |
| VT_F_TRANS_DUR | Float | 5.0 | 0.0 | 20.0 | s | Duration of a front transition |
| VT_F_TRANS_THR | Float | 1.0 | 0.0 | 1.0 | - | Target throttle value for the transition to fixed wing flight |
| VT_F_TR_OL_TM | Float | 6.0 | 1.0 | 30.0 | s | Airspeed less front transition time (open loop) |
| VT_IDLE_PWM_MC | Int32 | 900 | 900 | 2000 | us | Idle speed of VTOL when in multicopter mode |
| VT_MC_ON_FMU | Int32 | 0 | - | - | - | Enable the usage of AUX outputs for hover motors |
| VT_MOT_ID | Int32 | 0 | 0 | 12345678 | - | The channel number of motors which provide lift during hover |
| VT_PSHER_RMP_DT | Float | 3.0 | - | 20.0 | - | Pusher throttle ramp up window |
| VT_TILT_FW | Float | 1.0 | 0.0 | 1.0 | - | Position of tilt servo in fw mode |
| VT_TILT_MC | Float | 0.0 | 0.0 | 1.0 | - | Position of tilt servo in mc mode |
| VT_TILT_SPINUP | Float | 0.0 | 0.0 | 1.0 | - | Tilt actuator control value commanded when disarmed and during the first second after arming |
| VT_TILT_TRANS | Float | 0.3 | 0.0 | 1.0 | - | Position of tilt servo in transition mode |
| VT_TRANS_MIN_TM | Float | 2.0 | 0.0 | 20.0 | s | Front transition minimum time |
| VT_TRANS_P2_DUR | Float | 0.5 | 0.1 | 5.0 | s | Duration of front transition phase 2 |
| VT_TRANS_TIMEOUT | Float | 15.0 | 0.0 | 30.0 | s | Front transition timeout |
| VT_TYPE | Int32 | 0 | 0 | 2 | - | VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2) |
| WV_GAIN | Float | 1.0 | 0.0 | 3.0 | Hz | Weather-vane roll angle to yawrate |
