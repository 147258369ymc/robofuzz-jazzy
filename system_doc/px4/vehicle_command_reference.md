# PX4 VehicleCommand Reference

> Source: https://docs.px4.io/main/en/msg_docs/VehicleCommand

Vehicle Command uORB message used for commanding missions, actions, etc. Follows the MAVLink COMMAND_INT / COMMAND_LONG definition.

**Topics:** `vehicle_command`, `gimbal_v1_command`, `vehicle_command_mode_executor`

## Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| timestamp | uint64 | us | Time since system start |
| param1 | float32 | | Parameter 1, as defined by VEHICLE_CMD enum |
| param2 | float32 | | Parameter 2 |
| param3 | float32 | | Parameter 3 |
| param4 | float32 | | Parameter 4 |
| param5 | float64 | | Parameter 5 (often latitude) |
| param6 | float64 | | Parameter 6 (often longitude) |
| param7 | float32 | | Parameter 7 (often altitude) |
| command | uint32 | | Command ID |
| target_system | uint8 | | System which should execute the command |
| target_component | uint8 | | Component which should execute the command |
| source_system | uint8 | | System sending the command |
| source_component | uint16 | | Component sending the command |
| confirmation | uint8 | | 0=First transmission, 1-255=Confirmations |
| from_external | bool | | True if from external source |

## Command Definitions

### Navigation Commands

| Command | ID | Description |
|---------|------|-------------|
| VEHICLE_CMD_NAV_WAYPOINT | 16 | Navigate to waypoint |
| VEHICLE_CMD_NAV_LOITER_UNLIM | 17 | Loiter indefinitely |
| VEHICLE_CMD_NAV_LOITER_TURNS | 18 | Loiter for X turns |
| VEHICLE_CMD_NAV_LOITER_TIME | 19 | Loiter for specified time |
| VEHICLE_CMD_NAV_RETURN_TO_LAUNCH | 20 | Return to launch |
| VEHICLE_CMD_NAV_LAND | 21 | Land at location |
| VEHICLE_CMD_NAV_TAKEOFF | 22 | Takeoff from ground/hand |
| VEHICLE_CMD_NAV_PRECLAND | 23 | Precision landing |
| VEHICLE_CMD_DO_ORBIT | 34 | Start orbiting |
| VEHICLE_CMD_DO_FIGUREEIGHT | 35 | Figure eight pattern |
| VEHICLE_CMD_NAV_ROI | 80 | Set region of interest |
| VEHICLE_CMD_NAV_PATHPLANNING | 81 | Control path planning |
| VEHICLE_CMD_NAV_VTOL_TAKEOFF | 84 | VTOL takeoff + transition |
| VEHICLE_CMD_NAV_VTOL_LAND | 85 | VTOL transition + land |
| VEHICLE_CMD_NAV_GUIDED_LIMITS | 90 | Set external control limits |
| VEHICLE_CMD_NAV_DELAY | 93 | Delay next nav command |

### DO Commands

| Command | ID | Description |
|---------|------|-------------|
| VEHICLE_CMD_DO_SET_MODE | 176 | Set system mode |
| VEHICLE_CMD_DO_JUMP | 177 | Jump to mission command |
| VEHICLE_CMD_DO_CHANGE_SPEED | 178 | Change speed/throttle |
| VEHICLE_CMD_DO_SET_HOME | 179 | Change home location |
| VEHICLE_CMD_DO_SET_PARAMETER | 180 | Set a system parameter |
| VEHICLE_CMD_DO_SET_RELAY | 181 | Set relay condition |
| VEHICLE_CMD_DO_FLIGHTTERMINATION | 185 | Terminate flight immediately |
| VEHICLE_CMD_DO_CHANGE_ALTITUDE | 186 | Set to Loiter and change altitude |
| VEHICLE_CMD_DO_SET_ACTUATOR | 187 | Set actuators to desired value |
| VEHICLE_CMD_DO_LAND_START | 189 | Landing sequence marker |
| VEHICLE_CMD_DO_GO_AROUND | 191 | Abort autonomous landing |
| VEHICLE_CMD_DO_REPOSITION | 192 | Reposition to GPS location |
| VEHICLE_CMD_DO_PAUSE_CONTINUE | 193 | Pause/continue |
| VEHICLE_CMD_DO_SET_ROI_LOCATION | 195 | Set ROI to location |
| VEHICLE_CMD_DO_SET_ROI_NONE | 197 | Cancel ROI |
| VEHICLE_CMD_DO_CONTROL_VIDEO | 200 | Control camera system |
| VEHICLE_CMD_DO_SET_ROI | 201 | Set ROI for sensor/vehicle |
| VEHICLE_CMD_DO_MOUNT_CONFIGURE | 204 | Configure mount |
| VEHICLE_CMD_DO_MOUNT_CONTROL | 205 | Control mount |
| VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST | 206 | Set camera trigger distance |
| VEHICLE_CMD_DO_FENCE_ENABLE | 207 | Enable geofence |
| VEHICLE_CMD_DO_PARACHUTE | 208 | Trigger parachute |
| VEHICLE_CMD_DO_MOTOR_TEST | 209 | Motor test |
| VEHICLE_CMD_DO_INVERTED_FLIGHT | 210 | Inverted flight |
| VEHICLE_CMD_DO_GRIPPER | 211 | Operate gripper |
| VEHICLE_CMD_DO_AUTOTUNE_ENABLE | 212 | Enable autotune |
| VEHICLE_CMD_DO_SET_STANDARD_MODE | 262 | Enable standard MAVLink mode |

### System Commands

| Command | ID | Description |
|---------|------|-------------|
| VEHICLE_CMD_PREFLIGHT_CALIBRATION | 241 | Trigger calibration (pre-flight only) |
| VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS | 242 | Set sensor offsets |
| VEHICLE_CMD_PREFLIGHT_UAVCAN | 243 | UAVCAN configuration |
| VEHICLE_CMD_PREFLIGHT_STORAGE | 245 | Parameter/mission storage |
| VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN | 246 | Reboot or shutdown |
| VEHICLE_CMD_MISSION_START | 300 | Start running a mission |
| VEHICLE_CMD_COMPONENT_ARM_DISARM | 400 | Arm/Disarm component |
| VEHICLE_CMD_RUN_PREARM_CHECKS | 401 | Run pre-arm checks |
| VEHICLE_CMD_INJECT_FAILURE | 420 | Inject artificial failure |
| VEHICLE_CMD_REQUEST_MESSAGE | 512 | Request single message |
| VEHICLE_CMD_LOGGING_START | 2510 | Start ULog streaming |
| VEHICLE_CMD_LOGGING_STOP | 2511 | Stop ULog streaming |

### Key Command Parameter Details

#### VEHICLE_CMD_NAV_WAYPOINT (16)
| Param | Units | Description |
|-------|-------|-------------|
| 1 | s | Hold time |
| 2 | m | Acceptance radius |
| 3 | | 0=pass through WP; >0=radius to pass by (+=CW, -=CCW) |
| 4 | | Desired yaw angle at waypoint |
| 5 | | Latitude |
| 6 | | Longitude |
| 7 | | Altitude |

#### VEHICLE_CMD_NAV_TAKEOFF (22)
| Param | Units | Description |
|-------|-------|-------------|
| 4 | deg | Yaw angle in NED [0:360] |
| 5 | | Latitude (WGS-84) |
| 6 | | Longitude (WGS-84) |
| 7 | m | Altitude AMSL |

#### VEHICLE_CMD_DO_CHANGE_SPEED (178)
| Param | Units | Description |
|-------|-------|-------------|
| 1 | | Speed type (0=Airspeed, 1=Ground Speed) |
| 2 | m/s | Speed (-1 = no change) |
| 3 | % | Throttle (-1 = no change) |

#### VEHICLE_CMD_DO_SET_HOME (179)
| Param | Units | Description |
|-------|-------|-------------|
| 1 | | 1=current location, 0=specified |
| 5 | | Latitude |
| 6 | | Longitude |
| 7 | | Altitude |

#### VEHICLE_CMD_COMPONENT_ARM_DISARM (400)
| Param | Units | Description |
|-------|-------|-------------|
| 1 | | 1=arm, 0=disarm |

#### VEHICLE_CMD_CONDITION_YAW (115)
| Param | Units | Description |
|-------|-------|-------------|
| 1 | deg | Target angle [0:360] (0=north) |
| 2 | deg/s | Speed during yaw change |
| 3 | | Direction: negative=CCW, positive=CW |
| 4 | | 0=relative offset, 1=absolute angle |
