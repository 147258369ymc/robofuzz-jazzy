# PX4 Parameters & Configurations

## Overview

PX4 uses a "param subsystem" consisting of a flat table of `float` and `int32_t` values, along with text files for startup scripts, to store its configuration.

## Command Line Usage

The PX4 system console provides the `param` tool for managing parameters.

### Getting and Setting Parameters

- `param show` — lists all system parameters
- Wildcard filtering: `param show RC_MAP_A*`
- `param show -c` — shows parameters changed from defaults
- `param show-for-airframe` — shows parameters changed from the current airframe's defaults

### Exporting and Loading Parameters

**Saving:**
- `param save` — stores changed parameters to the default file
- `param save /fs/microsd/vtol_param_backup` — stores to a specific location

**Loading:**
- `param load` — resets all parameters to defaults first, then overwrites with file values
- `param import` — overwrites current values with file values without resetting, then saves

The `load` command resets parameters to the state when saved. The `import` command merges file parameters with the vehicle's current state — useful for importing calibration data without overwriting other configuration.

## Creating/Defining Parameters

Parameter definitions have two parts:
1. **Parameter metadata** — specifies default values and presentation information
2. **C/C++ Code** — provides access to get/subscribe to parameter values

### Parameter Names

- Maximum 16 ASCII characters
- By convention, parameters in a group share a meaningful prefix followed by underscore (e.g., `MC_` for multicopter, `FW_` for fixed-wing)
- Name must match between code and metadata

## C / C++ API

### C++ API

The C++ API declares parameters as class attributes with automatic synchronization via uORB. Required headers:

```cpp
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
```

Class definition using `DEFINE_PARAMETERS`:

```cpp
class MyModule : ..., public ModuleParams
{
public:
    ...
private:
    void parameters_update();

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,
        (ParamFloat<px4::params::ATT_BIAS_MAX>) _att_bias_max
    )

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
```

Update checking implementation:

```cpp
void Module::parameters_update()
{
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }
}
```

Key points:
- `_parameter_update_sub.updated()` checks for any parameter update
- `ModuleParams::updateParams()` syncs all attributes in `DEFINE_PARAMETERS`
- Better type-safety and less RAM overhead than C API
- Parameter name must be known at compile time

### C API

```c
#include <parameters/param.h>

int32_t my_param = 0;
param_get(param_find("PARAM_NAME"), &my_param);
```

For repeated reads, cache the handle:

```c
param_t my_param_handle = PARAM_INVALID;
my_param_handle = param_find("PARAM_NAME");

int32_t my_param = 0;
param_get(my_param_handle, &my_param);
```

`param_find()` is expensive; caching the handle is recommended for multiple reads.

## Parameter Metadata

Metadata can be stored as **.c** or **.yaml** files anywhere in the source tree, typically alongside the associated module. The build system extracts metadata using `make parameters_metadata`.

After adding a new parameter file, run `make clean` before building since parameter files are added during the cmake configure step.

### YAML Metadata

YAML is the newer, more flexible format (cannot currently be used in libraries).

- Schema: `validation/module_schema.yaml`
- Example: `/src/modules/mavlink/module.yaml`

Register in CMakeLists.txt:

```cmake
MODULE_CONFIG
    module.yaml
```

#### Multi-Instance (Templated) YAML

Use `${i}` for instance numbers:

```yaml
MY_PARAM_${i}_RATE:
  description:
    short: Maximum rate for instance ${i}
```

Configuration options:
- `num_instances` (default 1): number of instances to generate
- `instance_start` (default 0): first instance number

### C Parameter Metadata (.c files)

The legacy approach, still most common in the source tree:

```cpp
/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @reboot_required true
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);
```

```cpp
/**
 * Acceleration compensation based on GPS velocity.
 *
 * @group Attitude Q estimator
 * @boolean
 */
PARAM_DEFINE_INT32(ATT_ACC_COMP, 1);
```

The `PARAM_DEFINE_*` macro specifies:
- Type: `PARAM_DEFINE_FLOAT` or `PARAM_DEFINE_INT32`
- Parameter name (must match code usage)
- Default value in firmware

Comment block annotations:

| Annotation | Purpose |
|---|---|
| `@unit` | Unit (e.g., m for meters) |
| `@min` | Minimum sane value |
| `@max` | Maximum sane value |
| `@decimal` | Decimal places for display |
| `@increment` | UI tick increment |
| `@reboot_required true` | Requires system restart on change |
| `@boolean` | Integer parameter representing boolean |
| `@group` | Title for parameter group |

## Publishing Parameter Metadata to a GCS

The parameter metadata JSON file is compiled into firmware or hosted online, made available via the MAVLink Component Metadata service. This keeps metadata synchronized with the running vehicle code.

## Read-Only Parameters

For productized PX4 builds, parameters can be locked to prevent end-user modification of safety-critical settings.

### Configuration

Create `boards/<vendor>/<board>/readonly_params.yaml`:

```yaml
mode: block
parameters:
  - SYS_AUTOSTART
  - SYS_AUTOCONFIG
  - BAT1_N_CELLS
```

Two modes:
- **`block`**: listed parameters are read-only, all others writable
- **`allow`**: only listed parameters are writable, all others read-only

### Locking

`param lock` is called in `rcS` after all startup scripts finish. Before this call, startup scripts can freely set any parameter. To set a locked value, use `param set-default` in `rc.board_defaults` before the lock activates.

### Enforcement (after lock)

- `param set` / `param set-default` shell commands return errors
- MAVLink PARAM_SET returns `MAV_PARAM_ERROR_READ_ONLY`
- `param_set()` / `param_set_default_value()` C API calls return `PX4_ERROR`
- `param reset` silently skips read-only parameters
- `param import` / `param load` silently skips read-only parameters

The read-only list is compiled as a `constexpr` array with zero runtime overhead when empty.
