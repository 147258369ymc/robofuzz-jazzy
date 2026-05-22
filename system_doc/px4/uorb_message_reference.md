# PX4 uORB Message Reference

> Source: https://docs.px4.io/main/en/msg_docs/

PX4 internal module communication is based on uORB (Micro Object Request Broker). This document lists all available uORB messages used in PX4.

## Versioned Messages

These messages track changes to their definitions, with each modification resulting in a version increment. They are shared through the PX4-ROS 2 Bridge.

| Message | Description |
|---------|-------------|
| ActuatorMotors | Motor control message |
| ActuatorServos | Servo control message |
| AirspeedValidated | Validated airspeed |
| ArmingCheckReply | Arming check reply |
| ArmingCheckRequest | Arming check request |
| AuxGlobalPosition | Auxiliary global position |
| BatteryStatus | Battery status |
| ConfigOverrides | Configurable overrides by (external) modes or mode executors |
| Event | Events interface |
| FixedWingLateralSetpoint | Fixed Wing Lateral Setpoint |
| FixedWingLongitudinalSetpoint | Fixed Wing Longitudinal Setpoint |
| GotoSetpoint | Position and (optional) heading setpoints with speed constraints |
| HomePosition | GPS home position in WGS84 coordinates |
| LateralControlConfiguration | Fixed Wing Lateral Control Configuration |
| LongitudinalControlConfiguration | Fixed Wing Longitudinal Control Configuration |
| ManualControlSetpoint | Manual control input setpoint |
| ModeCompleted | Mode completion result, published by an active mode |
| RegisterExtComponentReply | External component registration reply |
| RegisterExtComponentRequest | Request to register an external component |
| TrajectorySetpoint | Trajectory setpoint in NED frame; input to PID position controller |
| UnregisterExtComponent | Unregister external component |
| VehicleAngularVelocity | Vehicle angular velocity |
| VehicleAttitude | Attitude quaternion (Hamilton, order w,x,y,z) for onboard use |
| VehicleAttitudeSetpoint | Vehicle attitude setpoint |
| VehicleCommand | Vehicle Command following MAVLink COMMAND_INT/COMMAND_LONG |
| VehicleCommandAck | Vehicle Command Acknowledgement |
| VehicleControlMode | Vehicle control mode flags |
| VehicleGlobalPosition | Fused global position in WGS84 from position estimator |
| VehicleLandDetected | Vehicle land detected state |
| VehicleLocalPosition | Fused local position in NED, origin at EKF2 start |
| VehicleOdometry | Vehicle odometry data |
| VehicleRatesSetpoint | Vehicle rates setpoint |
| VehicleStatus | System state of the vehicle published by commander |
| VtolVehicleStatus | VTOL state matching MAVLink's MAV_VTOL_STATE |
| Wind | Wind estimate from EKF2 |

## Unversioned Messages

| Message | Description |
|---------|-------------|
| ActionRequest | Action request for the vehicle's main state |
| ActuatorArmed | Actuator armed state |
| ActuatorControlsStatus | Actuator controls status |
| ActuatorOutputs | Actuator outputs |
| ActuatorServosTrim | Servo trims, added as offset to servo outputs |
| ActuatorTest | Actuator test |
| AdcReport | ADC raw data |
| Airspeed | Airspeed data from sensors |
| AirspeedWind | Wind estimate from airspeed_selector |
| AutotuneAttitudeControlStatus | Autotune attitude control status |
| BatteryInfo | Battery information |
| ButtonEvent | Button event |
| CameraCapture | Camera capture |
| CameraStatus | Camera status |
| CameraTrigger | Camera trigger |
| CanInterfaceStatus | CAN interface status |
| CellularStatus | Cellular status |
| CollisionConstraints | Local setpoint constraints in NED frame (NaN = no limit) |
| ControlAllocatorStatus | Control allocator status |
| Cpuload | CPU load |
| DebugArray | Debug array |
| DebugKeyValue | Debug key-value pair |
| DebugValue | Debug value |
| DebugVect | Debug vector |
| DifferentialPressure | Differential-pressure (airspeed) sensor |
| DistanceSensor | DISTANCE_SENSOR message data |
| Ekf2Timestamps | Relative timestamps of sensor inputs used by EKF2 |
| EscReport | ESC report |
| EscStatus | ESC status |
| EstimatorAidSource1d | Estimator aid source 1D |
| EstimatorAidSource2d | Estimator aid source 2D |
| EstimatorAidSource3d | Estimator aid source 3D |
| EstimatorBias | Estimator bias |
| EstimatorBias3d | Estimator bias 3D |
| EstimatorEventFlags | Estimator event flags |
| EstimatorGpsStatus | Estimator GPS status |
| EstimatorInnovations | Estimator innovations |
| EstimatorSensorBias | Sensor readings and in-run biases in SI-unit form |
| EstimatorStates | Estimator states |
| EstimatorStatus | Estimator status |
| EstimatorStatusFlags | Estimator status flags |
| FailsafeFlags | Input flags for the failsafe state machine |
| FailureDetectorStatus | Failure detector status |
| FollowTarget | Follow target |
| FollowTargetEstimator | Follow target estimator |
| FollowTargetStatus | Follow target status |
| GeofenceResult | Geofence result |
| GeofenceStatus | Geofence status |
| GimbalControls | Gimbal controls |
| GimbalDeviceAttitudeStatus | Gimbal device attitude status |
| GimbalDeviceInformation | Gimbal device information |
| GimbalDeviceSetAttitude | Gimbal device set attitude |
| GimbalManagerInformation | Gimbal manager information |
| GimbalManagerSetAttitude | Gimbal manager set attitude |
| GimbalManagerStatus | Gimbal manager status |
| GpsDump | Used to dump raw GPS communication to the log |
| GpsInjectData | GPS inject data |
| Gripper | Commands gripper actuation mapped to control allocation output |
| HealthReport | Health report |
| HeaterStatus | Heater status |
| HoverThrustEstimate | Hover thrust estimate |
| InputRc | Input RC |
| IridiumsbdStatus | Iridium SBD status |
| IrlockReport | IRLOCK_REPORT message data |
| LandingGear | Landing gear |
| LandingTargetPose | Relative position of precision land target |
| LaunchDetectionStatus | Status of launch detection state machine (fixed-wing) |
| LedControl | LED control for externally visible LEDs |
| LogMessage | Logging message output with PX4_WARN, PX4_ERR, PX4_INFO |
| ManualControlSwitches | Manual control switches |
| Mission | Mission |
| MissionResult | Mission result |
| NavigatorMissionItem | Navigator mission item |
| NavigatorStatus | Current status of a Navigator mode |
| ObstacleDistance | Obstacle distances in front of the sensor |
| OffboardControlMode | Off-board control mode |
| OnboardComputerStatus | ONBOARD_COMPUTER_STATUS message data |
| OrbitStatus | Orbit status |
| ParameterResetRequest | Used by primary to reset parameter value(s) on remote |
| ParameterSetValueRequest | Used to update a parameter value at the other end |
| ParameterSetValueResponse | Response to a set value request |
| ParameterUpdate | Notifies the system about parameter changes |
| PositionSetpoint | Used in position_setpoint triple as a dependency |
| PositionSetpointTriplet | Global position setpoint triplet in WGS84 |
| PowerMonitor | Power monitor message |
| RadioStatus | Radio status |
| RateCtrlStatus | Rate control status |
| RcChannels | RC channels |
| RoverAttitudeSetpoint | Rover Attitude Setpoint |
| RoverPositionSetpoint | Rover Position Setpoint |
| RoverRateSetpoint | Rover Rate setpoint |
| RoverSpeedSetpoint | Rover Speed Setpoint |
| Rpm | RPM |
| RtlStatus | RTL status |
| RtlTimeEstimate | RTL time estimate |
| SatelliteInfo | Satellite info |
| SensorAccel | Sensor accelerometer |
| SensorBaro | Sensor barometer |
| SensorCombined | Combined sensor data |
| SensorGps | Sensor GPS |
| SensorGyro | Sensor gyroscope |
| SensorMag | Sensor magnetometer |
| SensorOpticalFlow | Sensor optical flow |
| TakeoffStatus | Takeoff status |
| TelemetryStatus | Telemetry status |
| TimesyncStatus | Timesync status |
| TransponderReport | Transponder report |
| TuneControl | Tune control |
| UlogStream | ULog stream |
| VehicleAcceleration | Vehicle acceleration |
| VehicleAirData | Vehicle air data |
| VehicleConstraints | Vehicle constraints |
| VehicleImu | Vehicle IMU |
| VehicleMagnetometer | Vehicle magnetometer |
| VehicleThrustSetpoint | Vehicle thrust setpoint |
| VehicleTorqueSetpoint | Vehicle torque setpoint |

---

## Key Message Details

### VehicleStatus

Encodes the system state of the vehicle published by commander.

**Topic:** `vehicle_status`

| Field | Type | Description |
|-------|------|-------------|
| timestamp | uint64 | Time since system start (us) |
| armed_time | uint64 | Arming timestamp (us) |
| takeoff_time | uint64 | Takeoff timestamp (us) |
| arming_state | uint8 | Current arming state |
| latest_arming_reason | uint8 | Reason for most recent arming event |
| latest_disarming_reason | uint8 | Reason for most recent disarming event |
| nav_state_timestamp | uint64 | Time when current nav_state activated |
| nav_state_user_intention | uint8 | Mode the user selected |
| nav_state | uint8 | Currently active mode |
| executor_in_charge | uint8 | Current mode executor (0=Autopilot) |
| valid_nav_states_mask | uint32 | Bitmask for all valid nav_state values |
| can_set_nav_states_mask | uint32 | Bitmask for user-selectable modes |
| hil_state | uint8 | Hardware-in-the-loop state |
| vehicle_type | uint8 | Current vehicle locomotion method |
| failsafe | bool | True if system is in failsafe state |
| failsafe_and_user_took_over | bool | True if in failsafe but user took over |
| gcs_connection_lost | bool | Datalink to GCS lost |
| is_vtol | bool | True if the system is VTOL capable |
| is_vtol_tailsitter | bool | True if 90deg pitch down in MC-FW transition |
| in_transition_mode | bool | True if VTOL is doing a transition |
| in_transition_to_fw | bool | True if transitioning MC to FW |
| system_type | uint8 | MAVLink MAV_TYPE |
| system_id | uint8 | MAVLink system ID |
| component_id | uint8 | MAVLink component ID |
| safety_button_available | bool | Safety button connected |
| safety_off | bool | Safety is off |
| pre_flight_checks_pass | bool | All checks necessary to arm pass |

**Arming States:**
- ARMING_STATE_DISARMED = 1
- ARMING_STATE_ARMED = 2

**Navigation States:**
- NAVIGATION_STATE_MANUAL = 0
- NAVIGATION_STATE_ALTCTL = 1
- NAVIGATION_STATE_POSCTL = 2
- NAVIGATION_STATE_AUTO_MISSION = 3
- NAVIGATION_STATE_AUTO_LOITER = 4
- NAVIGATION_STATE_AUTO_RTL = 5
- NAVIGATION_STATE_ACRO = 10
- NAVIGATION_STATE_DESCEND = 12
- NAVIGATION_STATE_TERMINATION = 13
- NAVIGATION_STATE_OFFBOARD = 14
- NAVIGATION_STATE_STAB = 15
- NAVIGATION_STATE_AUTO_TAKEOFF = 17
- NAVIGATION_STATE_AUTO_LAND = 18
- NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
- NAVIGATION_STATE_AUTO_PRECLAND = 20
- NAVIGATION_STATE_ORBIT = 21
- NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22
- NAVIGATION_STATE_EXTERNAL1-8 = 23-30
- NAVIGATION_STATE_MAX = 31

**Vehicle Types:**
- VEHICLE_TYPE_UNSPECIFIED = 0
- VEHICLE_TYPE_ROTARY_WING = 1
- VEHICLE_TYPE_FIXED_WING = 2
- VEHICLE_TYPE_ROVER = 3
