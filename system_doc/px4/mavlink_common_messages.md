# MAVLink Common Messages Reference

> Source: https://mavlink.io/en/messages/common.html
> XML Definition: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml

## HEARTBEAT (ID: 0)

Shows that a system or component is present and responding.

| Field | Type | Description |
|-------|------|-------------|
| type | uint8_t | Vehicle or component type (MAV_TYPE enum) |
| autopilot | uint8_t | Autopilot type/class (MAV_AUTOPILOT enum) |
| base_mode | uint8_t | System mode bitmap (MAV_MODE_FLAG) |
| custom_mode | uint32_t | Autopilot-specific flags bitfield |
| system_status | uint8_t | System status flag (MAV_STATE) |
| mavlink_version | uint8_t | MAVLink version, added by protocol automatically |

## SYS_STATUS (ID: 1)

Compact representation of sensor/subsystem status and basic statistics.

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| onboard_control_sensors_present | uint32_t | | Bitmap of present onboard controllers/sensors |
| onboard_control_sensors_enabled | uint32_t | | Bitmap of enabled onboard controllers/sensors |
| onboard_control_sensors_health | uint32_t | | Bitmap of healthy onboard controllers/sensors |
| load | uint16_t | d% | Max mainloop time usage [0-1000] |
| voltage_battery | uint16_t | mV | Battery voltage (UINT16_MAX if not sent) |
| current_battery | int16_t | cA | Battery current (-1 if not sent) |
| battery_remaining | int8_t | % | Battery energy remaining (-1 if not sent) |
| drop_rate_comm | uint16_t | c% | Communication drop rate on all links |
| errors_comm | uint16_t | | Communication errors on all links |
| errors_count1-4 | uint16_t | | Autopilot-specific errors |

## SYSTEM_TIME (ID: 2)

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_unix_usec | uint64_t | us | Timestamp (UNIX epoch time) |
| time_boot_ms | uint32_t | ms | Timestamp (time since system boot) |

## SET_MODE (ID: 11) [DEPRECATED]

Replaced by MAV_CMD_DO_SET_MODE via COMMAND_LONG.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | The system setting the mode |
| base_mode | uint8_t | New base mode (MAV_MODE enum) |
| custom_mode | uint32_t | New autopilot-specific mode |

## PARAM_REQUEST_READ (ID: 20)

Request to read an onboard parameter by string ID.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| param_id | char[16] | Parameter id, NULL-terminated if <16 chars |
| param_index | int16_t | Parameter index (-1 to use param_id) |

## PARAM_REQUEST_LIST (ID: 21)

Request all parameters of a component.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |

## PARAM_VALUE (ID: 22)

Emits the value of an onboard parameter.

| Field | Type | Description |
|-------|------|-------------|
| param_id | char[16] | Parameter id, NULL-terminated if <16 chars |
| param_value | float | Onboard parameter value |
| param_type | uint8_t | Parameter type (MAV_PARAM_TYPE) |
| param_count | uint16_t | Total number of onboard parameters |
| param_index | uint16_t | Index of this parameter |

## PARAM_SET (ID: 23)

Write a new parameter value to permanent storage.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| param_id | char[16] | Parameter id, NULL-terminated if <16 chars |
| param_value | float | Onboard parameter value |
| param_type | uint8_t | Parameter type (MAV_PARAM_TYPE) |

## GPS_RAW_INT (ID: 24)

Raw GPS global position (not fused estimate).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_usec | uint64_t | us | Timestamp |
| fix_type | uint8_t | | GPS fix type (GPS_FIX_TYPE) |
| lat | int32_t | degE7 | Latitude (WGS84) |
| lon | int32_t | degE7 | Longitude (WGS84) |
| alt | int32_t | mm | Altitude (MSL), positive up |
| eph | uint16_t | 1E-2 | HDOP (unitless x100) |
| epv | uint16_t | 1E-2 | VDOP (unitless x100) |
| vel | uint16_t | cm/s | Ground speed |
| cog | uint16_t | cdeg | Course over ground [0.0..359.99] |
| satellites_visible | uint8_t | | Number of satellites visible |

## ATTITUDE (ID: 30)

Attitude in aeronautical frame (right-handed, Z-down, ZYX intrinsic).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_boot_ms | uint32_t | ms | Timestamp |
| roll | float | rad | Roll angle (-pi..+pi) |
| pitch | float | rad | Pitch angle (-pi..+pi) |
| yaw | float | rad | Yaw angle (-pi..+pi) |
| rollspeed | float | rad/s | Roll angular speed |
| pitchspeed | float | rad/s | Pitch angular speed |
| yawspeed | float | rad/s | Yaw angular speed |

## LOCAL_POSITION_NED (ID: 32)

Filtered local position in NED frame (Z-axis down).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_boot_ms | uint32_t | ms | Timestamp |
| x | float | m | X Position |
| y | float | m | Y Position |
| z | float | m | Z Position |
| vx | float | m/s | X Speed |
| vy | float | m/s | Y Speed |
| vz | float | m/s | Z Speed |

## GLOBAL_POSITION_INT (ID: 33)

Filtered global position (fused GPS + accelerometers).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_boot_ms | uint32_t | ms | Timestamp |
| lat | int32_t | degE7 | Latitude |
| lon | int32_t | degE7 | Longitude |
| alt | int32_t | mm | Altitude (MSL) |
| relative_alt | int32_t | mm | Altitude above home |
| vx | int16_t | cm/s | Ground X Speed (positive north) |
| vy | int16_t | cm/s | Ground Y Speed (positive east) |
| vz | int16_t | cm/s | Ground Z Speed (positive down) |
| hdg | uint16_t | cdeg | Vehicle heading [0.0..359.99] |

## COMMAND_INT (ID: 75)

Sends a command with up to seven parameters, where params 5 and 6 are integers. Preferred over COMMAND_LONG for positional information.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| frame | uint8_t | Coordinate system (MAV_FRAME) |
| command | uint16_t | Scheduled action (MAV_CMD) |
| current | uint8_t | Not used |
| autocontinue | uint8_t | Not used (set 0) |
| param1 | float | PARAM1 (NaN=invalid) |
| param2 | float | PARAM2 (NaN=invalid) |
| param3 | float | PARAM3 (NaN=invalid) |
| param4 | float | PARAM4 (NaN=invalid) |
| x | int32_t | PARAM5: local x (m*1e4) or lat (degE7) |
| y | int32_t | PARAM6: local y (m*1e4) or lon (degE7) |
| z | float | PARAM7: altitude in meters |

## COMMAND_LONG (ID: 76)

Sends a command with up to seven float parameters.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System which should execute the command |
| target_component | uint8_t | Component (0 for all) |
| command | uint16_t | Command ID (MAV_CMD) |
| confirmation | uint8_t | 0=First transmission, 1-255=Confirmations |
| param1 | float | Parameter 1 (NaN=invalid) |
| param2 | float | Parameter 2 (NaN=invalid) |
| param3 | float | Parameter 3 (NaN=invalid) |
| param4 | float | Parameter 4 (NaN=invalid) |
| param5 | float | Parameter 5 (NaN=invalid) |
| param6 | float | Parameter 6 (NaN=invalid) |
| param7 | float | Parameter 7 (NaN=invalid) |

## COMMAND_ACK (ID: 77)

Reports status of a command execution.

| Field | Type | Description |
|-------|------|-------------|
| command | uint16_t | Command ID (MAV_CMD) |
| result | uint8_t | Result (MAV_RESULT) |
| progress (ext) | uint8_t | Progress % when MAV_RESULT_IN_PROGRESS [0-100] |
| result_param2 (ext) | int32_t | Additional result info |
| target_system (ext) | uint8_t | Target recipient system ID |
| target_component (ext) | uint8_t | Target recipient component ID |

## MISSION_REQUEST_LIST (ID: 43)

Request overall list of mission items.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| mission_type (ext) | uint8_t | Mission type (MAV_MISSION_TYPE) |

## MISSION_COUNT (ID: 44)

Response to MISSION_REQUEST_LIST / initiate write transaction.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| count | uint16_t | Number of mission items |
| mission_type (ext) | uint8_t | Mission type |

## MISSION_ITEM_INT (ID: 73)

Encodes a mission item with integer lat/lon.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| seq | uint16_t | Waypoint ID (sequence number) |
| frame | uint8_t | Coordinate system (MAV_FRAME) |
| command | uint16_t | Scheduled action (MAV_CMD) |
| current | uint8_t | false:0, true:1 |
| autocontinue | uint8_t | 0=false, 1=true |
| param1-4 | float | Command-specific parameters |
| x | int32_t | Latitude (degE7) or local x (m*1e4) |
| y | int32_t | Longitude (degE7) or local y (m*1e4) |
| z | float | Altitude (meters) |
| mission_type (ext) | uint8_t | Mission type |

## MISSION_ACK (ID: 47)

Acknowledgment during waypoint handling.

| Field | Type | Description |
|-------|------|-------------|
| target_system | uint8_t | System ID |
| target_component | uint8_t | Component ID |
| type | uint8_t | Mission result (MAV_MISSION_RESULT) |
| mission_type (ext) | uint8_t | Mission type |

## SET_POSITION_TARGET_LOCAL_NED (ID: 84)

Sets desired vehicle position in local NED frame (external controller).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_boot_ms | uint32_t | ms | Timestamp |
| target_system | uint8_t | | System ID |
| target_component | uint8_t | | Component ID |
| coordinate_frame | uint8_t | | MAV_FRAME (1=LOCAL_NED, 7=OFFSET, 8=BODY) |
| type_mask | uint16_t | | POSITION_TARGET_TYPEMASK |
| x | float | m | X Position in NED |
| y | float | m | Y Position in NED |
| z | float | m | Z Position in NED (negative = up) |
| vx | float | m/s | X velocity |
| vy | float | m/s | Y velocity |
| vz | float | m/s | Z velocity |
| afx | float | m/s^2 | X acceleration/force |
| afy | float | m/s^2 | Y acceleration/force |
| afz | float | m/s^2 | Z acceleration/force |
| yaw | float | rad | Yaw setpoint |
| yaw_rate | float | rad/s | Yaw rate setpoint |

## SET_POSITION_TARGET_GLOBAL_INT (ID: 86)

Sets desired vehicle position in global WGS84 frame (external controller).

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| time_boot_ms | uint32_t | ms | Timestamp |
| target_system | uint8_t | | System ID |
| target_component | uint8_t | | Component ID |
| coordinate_frame | uint8_t | | MAV_FRAME (0=GLOBAL, 3=REL_ALT, 10=TERRAIN) |
| type_mask | uint16_t | | POSITION_TARGET_TYPEMASK |
| lat_int | int32_t | degE7 | Latitude (WGS84) |
| lon_int | int32_t | degE7 | Longitude (WGS84) |
| alt | float | m | Altitude (frame-dependent) |
| vx | float | m/s | X velocity in NED |
| vy | float | m/s | Y velocity in NED |
| vz | float | m/s | Z velocity in NED |
| afx | float | m/s^2 | X acceleration/force |
| afy | float | m/s^2 | Y acceleration/force |
| afz | float | m/s^2 | Z acceleration/force |
| yaw | float | rad | Yaw setpoint |
| yaw_rate | float | rad/s | Yaw rate setpoint |

## STATUSTEXT (ID: 253)

Status text message with severity level.

| Field | Type | Description |
|-------|------|-------------|
| severity | uint8_t | Severity (MAV_SEVERITY) |
| text | char[50] | Status text message |
| id (ext) | uint16_t | Unique identifier for reassembly |
| chunk_seq (ext) | uint8_t | Chunk sequence number |

## BATTERY_STATUS (ID: 147)

Battery information.

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| id | uint8_t | | Battery ID |
| battery_function | uint8_t | | Function (MAV_BATTERY_FUNCTION) |
| type | uint8_t | | Chemistry type (MAV_BATTERY_TYPE) |
| temperature | int16_t | cdegC | Temperature (INT16_MAX=unknown) |
| voltages | uint16_t[10] | mV | Cell voltages 1-10 |
| current_battery | int16_t | cA | Battery current (-1=not sent) |
| current_consumed | int32_t | mAh | Consumed charge (-1=not sent) |
| energy_consumed | int32_t | hJ | Consumed energy (-1=not sent) |
| battery_remaining | int8_t | % | Remaining energy [0-100], -1=not provided |

---

## Key Enums

### MAV_RESULT (COMMAND_ACK result)
- 0: MAV_RESULT_ACCEPTED — Command accepted
- 1: MAV_RESULT_TEMPORARILY_REJECTED — Temporarily rejected
- 2: MAV_RESULT_DENIED — Denied
- 3: MAV_RESULT_UNSUPPORTED — Unknown/unsupported
- 4: MAV_RESULT_FAILED — Failed
- 5: MAV_RESULT_IN_PROGRESS — In progress
- 6: MAV_RESULT_CANCELLED — Cancelled

### MAV_STATE (HEARTBEAT system_status)
- 0: MAV_STATE_UNINIT — Uninitialized
- 1: MAV_STATE_BOOT — Booting up
- 2: MAV_STATE_CALIBRATING — Calibrating
- 3: MAV_STATE_STANDBY — Standby, ready to fly
- 4: MAV_STATE_ACTIVE — Active (motors engaged)
- 5: MAV_STATE_CRITICAL — Critical state
- 6: MAV_STATE_EMERGENCY — Emergency
- 7: MAV_STATE_POWEROFF — Powering off
- 8: MAV_STATE_FLIGHT_TERMINATION — Flight termination

### GPS_FIX_TYPE
- 0: No GPS
- 1: No fix
- 2: 2D fix
- 3: 3D fix
- 4: DGPS
- 5: RTK float
- 6: RTK fixed

### POSITION_TARGET_TYPEMASK
- Bit 0: Ignore x position
- Bit 1: Ignore y position
- Bit 2: Ignore z position
- Bit 3: Ignore vx velocity
- Bit 4: Ignore vy velocity
- Bit 5: Ignore vz velocity
- Bit 6: Ignore ax acceleration
- Bit 7: Ignore ay acceleration
- Bit 8: Ignore az acceleration
- Bit 9: Use force instead of acceleration
- Bit 10: Ignore yaw
- Bit 11: Ignore yaw rate
