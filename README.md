# soil_modbus_rtu

Standalone UART Modbus RTU client library for CWT THC-S soil sensors.

## Layout

- `include/soil_modbus_rtu.h`: public API.
- `src/soil_modbus_rtu.c`: implementation.
- `CMakeLists.txt`: standalone CMake target (`soil_modbus_rtu`).

## Notes

- The library is MCU/RTOS-specific and currently uses Ambiq HAL + FreeRTOS APIs.
- RS-485 direction control (DE pin) is supported.
- Both polling and interrupt-driven UART read paths are implemented.

## Using In Another Repository

1. Copy this folder into the destination repository, or split it as its own repo.
2. Add `soil_modbus_rtu/include` to include paths.
3. Compile/link `soil_modbus_rtu/src/soil_modbus_rtu.c`.

## Splitting This Folder Into Its Own Git Repository

Example from project root:

```sh
git subtree split --prefix soil_modbus_rtu -b codex/soil_modbus_rtu-export
```

Then push `codex/soil_modbus_rtu-export` to a new remote repository.
