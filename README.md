# MPU-9250 and MPU-6500 Driver for TI-RTOS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive, easy-to-use I2C driver for the MPU-9250 IMU and MPU-6500 IMU, specifically tailored for the **TI-RTOS** environment on the **MSP-EXP432E401Y** platform. This library offers multiple levels of abstraction to get you up and running quickly.

This library provides everything you need to go from raw sensor readings to a fully calibrated, high-performance orientation solution.

## Features

- **Multi-Level API:** Choose the abstraction level that fits your needs:
  - A simple C library for direct register access and raw data (Work In Progress, needs refactoring).
  - A high-level C++ class that encapsulates the complex InvenSense DMP.
  - A C-compatible wrapper for using the DMP from any C project.
- **Sensor Fusion Options:** Includes a lightweight **Complementary Filter** and a full implementation of the onboard **Digital Motion Processor (DMP)** for 6-axis quaternion output.
- **TI-RTOS Integration:** Designed from the ground up for a real-time, task-based environment, using TI Drivers for I2C and TI-RTOS primitives for timing.

## Library Versions & Getting Started

The library is split into three parts, located in the `lib/` directory.

Read the full [API reference](https://bubi2001.github.io/MPU9250DMP-TI-RTOS/) for specific methods and functions availables

### 1. Base C Library (`MPU9250.h`/`.c`)

This is a low-level C driver for initializing the sensor and getting raw, calibrated data. It also includes a basic complementary filter.

Currently unavailable

### 2. C++ DMP Wrapper (`MPU_DMP.h`/`.cpp`)

A modern C++ class that wraps the complex, proprietary InvenSense DMP driver, providing a clean, object-oriented API. This is the recommended way to use the DMP in a C++ project.

Example Usage:

```cpp
#include "lib/MPU_DMP.h"
#include "lib/util/inv_mpu.h"
#include "lib/util/inv_mpu_dmp_motion_driver.h"

MPU_DMP dmp;
dmp.begin();
dmp.dmpBegin(DMP_FEATURE_6X_LP_QUAT, 100); // 100 Hz FIFO rate

while(1) {
    if (dmp.dmpUpdateFifo() == INV_SUCCESS) {
        dmp.computeEulerAngles();
        // Now use dmp.roll, dmp.pitch, dmp.yaw
    }
    Task_sleep(10);
}
```

### 3. C-Compatible DMP Wrapper (`MPU_DMP_C.h`/`.cpp`)

A C-style wrapper that allows you to safely use the C++ DMP class from a standard C project using an opaque pointer.

Example Usage:

```c
#include "lib/MPU_DMP_C.h"
#include "lib/util/inv_mpu.h"
#include "lib/util/inv_mpu_dmp_motion_driver.h"

MPU_DMP_Handle dmp_handle = MPU_DMP_Create();
MPU_DMP_begin(dmp_handle);
// ... setup DMP ...

while(1) {
    if (MPU_DMP_dmpUpdateFifo(dmp_handle) == 0) {
        // Get quaternion data and compute angles...
    }
    Task_sleep(10);
}
MPU_DMP_Destroy(dmp_handle);
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

The core DMP driver is based on the work by InvenSense/TDK.

The C++ wrapper is an adaptation of the excellent [SparkFun MPU-9250 DMP Arduino Library](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library).
