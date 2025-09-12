# MPU-6500 and MPU-9250 Driver for TI-RTOS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive, easy-to-use I2C driver for the MPU-6500 and the MPU-9250 IMU, specifically tailored for the **TI-RTOS** environment on the **MSP-EXP432E401Y** platform. This library offers multiple levels of abstraction to get you up and running quickly.

This library provides everything you need to go from raw sensor readings to a fully calibrated, high-performance orientation solution.

## Features

- **Multi-Level API:** Choose the abstraction level that fits your needs:
  - A C library for direct register access, raw data and a complete sensor fusion hub.
  - A high-level C++ class that encapsulates the complex InvenSense DMP.
  - A C-compatible wrapper for using the DMP from any C project.
- **Sensor Fusion Options:** Includes a full implementation of the onboard **Digital Motion Processor (DMP)** for 6-axis quaternion output. The low-level C library includes options for simple **Complementary Filter** as well as advanced  **Madgwick and Mahony Filters** for either 6-axis or 9-axis quaternion output, with the possibility to convert to Euler Angles directly.
- **TI-RTOS Integration:** Designed from the ground up for a real-time, task-based environment, using TI Drivers for I2C and TI-RTOS primitives for timing.

## License

This project is licensed under the MIT License.

## Acknowledgements

The core DMP driver is based on the work by InvenSense/TDK.

The C++ wrapper is an adaptation of the excellent [SparkFun MPU-9250 DMP Arduino Library](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library).
