# MAVLink Parameter Protocol

> Source: https://mavlink.io/en/services/parameter.html

## Overview

The parameter microservice enables configuration settings exchange between MAVLink components. Parameters are key/value pairs where the key is a human-readable name (max 16 characters) and the value can be one of several types defined in `MAV_PARAM_TYPE`.

Key properties:
- Human-readable names allow users to infer parameter purpose
- Unknown autopilots implementing the protocol work out of the box
- A GCS doesn't need advance knowledge of remote system parameters
- Adding parameters only requires changes on the system with those parameters

## Messages and Enums

| Message | Purpose |
|---------|---------|
| `PARAM_REQUEST_LIST` | Request all parameters; recipient broadcasts all via `PARAM_VALUE` |
| `PARAM_REQUEST_READ` | Request a single parameter by name or index |
| `PARAM_SET` | Command to set a parameter to a specified value |
| `PARAM_VALUE` | Broadcasts current value in response to requests or after changes |

| Enum/Flag | Purpose |
|-----------|---------|
| `MAV_PARAM_TYPE` | Indicates the real type of the encoded parameter value |
| `MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE` | Byte-wise encoding |
| `MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST` | C-style cast encoding |

## Parameter Names

The `param_id` field stores up to 16 characters. NULL-terminated if shorter than 16 characters, no null termination if exactly 16 characters.

## Parameter Encoding

Values are encoded in `param_value`, an IEEE754 single-precision 4-byte float field. The `param_type` field indicates the actual data type.

### Byte-wise Encoding (Recommended)

The parameter's bytes are copied directly into the float field's bytes. Preserves precision for large integers (e.g., 1E7 scaled integers).

```c
mavlink_param_union_t param;
int32_t integer = 20000;
param.param_int32 = integer;
param.type = MAV_PARAM_TYPE_INT32;
mavlink_msg_param_set_send(xxx, xxx, param.param_float, param.type, xxx);
```

### C-style Cast Encoding

The parameter value is converted to a float, which may lose precision since floats only represent integers with up to 24 bits of precision.

## Parameter Types (MAV_PARAM_TYPE)

Supported types: 8, 16, 32, and 64-bit signed/unsigned integers, plus 32 and 64-bit floats.

**PX4 only uses INT32 and FLOAT.**

## Parameter Operations

### Read All Parameters

1. GCS sends `PARAM_REQUEST_LIST` with target system/component
2. Target broadcasts all parameters individually via `PARAM_VALUE`
3. GCS starts a receive timeout after each `PARAM_VALUE`
4. `PARAM_VALUE` contains `param_count` (total) and `param_index` (current)
5. GCS can detect missing parameters and request them individually

### Read Single Parameter

1. GCS sends `PARAM_REQUEST_READ` specifying `param_id` (name) or `param_index`
2. Drone responds with `PARAM_VALUE` (broadcast to all systems)
3. GCS may re-request on timeout

### Write Parameters

1. GCS sends `PARAM_SET` with param name, new value, target, and param type
2. Drone writes the parameter and broadcasts `PARAM_VALUE` with updated value
3. Drone must broadcast `PARAM_VALUE` even if write fails (returning unchanged value)
4. GCS updates its cache with the received value
5. GCS may retry on timeout or if returned value doesn't match

## PX4 Implementation Details

- Uses **byte-wise encoding** (though `MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE` is not set as of v1.12)
- Only **float and Int32** parameter types are used
- **Caching extension**: When parameters are requested, PX4 first sends a `PARAM_VALUE` with `param_index` set to `INT16_MAX` (called `PARAM_HASH`) containing a CRC32 hash computed over all param names and values. If the GCS has a matching hash, it can use its cached parameters immediately.

Source: `src/modules/mavlink/mavlink_parameters.cpp`

## Parameter Table Invariance

The protocol requires that the parameter set does not change during normal operation after initial read. There is no mechanism to notify that the set has changed.

## Multi-System and Multi-Component Support

- Requests can target individual systems or components
- Setting `target_component` to `MAV_COMP_ID_ALL` retrieves parameters from all components
- All components must respond to requests addressed to their ID or `MAV_COMP_ID_ALL`
