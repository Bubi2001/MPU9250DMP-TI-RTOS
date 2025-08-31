# MPU6500/9250 DMP Driver API Reference

This document provides a detailed API reference for the `MPU_DMP` driver, which interfaces with InvenSense MPU6500 and MPU9250 inertial measurement units (IMUs). The driver simplifies the use of the sensor's on-board Digital Motion Processor (DMP) to retrieve stable, quaternion-based orientation data.

The library is written in C++ but provides a complete C wrapper API for use in C-only projects. This is achieved through an opaque pointer (MPU_DMP_Handle), which conceals the underlying C++ object implementation.

## Key Features

* Simplified initialization of the MPU and DMP.
* Easy configuration of sensor parameters like Full-Scale Range (FSR) and sample rates.
* DMP feature support, including 6-axis quaternion fusion, tap detection, and pedometer.
* Direct access to raw sensor data and DMP-processed quaternions.
* Helper functions to convert raw data and calculate Euler angles (pitch, roll, yaw).
* Conditional support for the MPU9250's magnetometer.

---

## C++ API Reference (`MPU_DMP` Class)

This is the core object-oriented interface. To use it, include `MPU_DMP.h` in your project.

### Public Data Members

After a successful call to `dmpUpdateFifo()`, these members are populated with the latest data from the sensor.

* `int ax, ay, az;`
  * Raw accelerometer data for X, Y, and Z axes.
* `int gx, gy, gz;`
  * Raw gyroscope data for X, Y, and Z axes.
* `long qw, qx, qy, qz;`
  * Raw quaternion data (W, X, Y, Z) from the DMP in Q30 fixed-point format.
* `unsigned long time;`
  * Timestamp for the last DMP data packet.
* `float pitch, roll, yaw;`
  * Calculated Euler angles. These are only updated after you call `computeEulerAngles()`.

### MPU9250 Specific Members

These members are only available if the `USE_MPU9250` macro is defined.

* `int mx, my, mz;`
  * Raw magnetometer data. **Note:** This is not read from the DMP FIFO and must be fetched separately.

* `float heading;`
  * Calculated compass heading. Updated by calling `computeCompassHeading()`.

---

### Core Methods

#### Initialization & Status

`MPU_DMP()`

* **Description:** The class constructor. Initializes internal variables.

`int begin()`

* **Description:** Initializes the MPU sensor hardware and the underlying motion driver. This must be called before any other function.
* **Returns:** `INV_SUCCESS` (0) on success, `INV_ERROR` (-1) on failure.

`int selfTest()`

* **Description:** Runs the sensor's built-in self-test routine to verify hardware functionality.
* **Returns:** A bitmask of test results. For MPU6500, a result of `0x03` means gyro and accel passed. For MPU9250, a result of `0x07` means all three sensors passed.

#### DMP (Digital Motion Processor) Functions

`int dmpBegin(unsigned short features, unsigned short fifoRate = 100)`

* **Description**: Loads the DMP firmware, enables specified features, and starts the DMP. This is the primary function to begin motion processing.
* **Parameters**:
  * `features`: A bitmask of DMP features to enable. Available features:
    * `DMP_FEATURE_LP_QUAT`: Generates quaternions from the gyroscope data only (3-axis fusion). This is less accurate but consumes less power.
    * `DMP_FEATURE_6X_LP_QUAT`: Generates quaternions using data from both the gyroscope and accelerometer (6-axis sensor fusion). This provides a more stable orientation and corrects for gyro drift. This is the most commonly used feature for orientation tracking.
    * `DMP_FEATURE_TAP`: Enables tap detection. When a tap is detected, the DMP will generate an interrupt.
    * `DMP_FEATURE_ANDROID_ORIENT`: Enables the DMP to determine the screen orientation (portrait, landscape, etc.) based on gravity, similar to an Android phone.
    * `DMP_FEATURE_PEDOMETER`: Enables the built-in step counter.
    * `DMP_FEATURE_GYRO_CAL`: Enables continuous calibration of the gyroscope at runtime, helping to reduce drift.
    * `DMP_FEATURE_SEND_RAW_ACCEL`: Sends raw accelerometer data to the FIFO buffer.
    * `DMP_FEATURE_SEND_RAW_GYRO`: Sends raw gyroscope data to the FIFO buffer.
    * `DMP_FEATURE_SEND_CAL_GYRO`: Sends calibrated gyroscope data to the FIFO buffer.
  * `fifoRate`: The desired rate in Hz for the DMP to generate new data packets (e.g., 100 Hz).
* **Returns**: `INV_SUCCESS` (0) on success, `INV_ERROR` (-1) on failure.

`int dmpUpdateFifo()`

* **Description**: Reads the latest data packet from the DMP's FIFO buffer. This function should be called repeatedly in your main loop. If a new packet is available, it updates the public data members (`ax`, `ay`, `az`, `gx`, `gy`, `gz`, `qw`, `qx`, `qy`, `qz`).
* **Returns**: `INV_SUCCESS` (0) if a new packet was read, `INV_ERROR` (-1) if no new data was available.

#### Data Calculation & Conversion

`void computeEulerAngles(bool degrees = true)`

* **Description**: Computes pitch, roll, and yaw from the current quaternion data (`qw`, `qx`, `qy`, `qz`). The results are stored in the public `pitch`, `roll`, and `yaw` members.
* **Parameters**:
  * `degrees`: If `true` (default), the results are in degrees. If `false`, they are in radians.

`float calcAccel(int axis_val)`

* **Description**: Converts a raw accelerometer integer value to physical units (g's).
* **Returns**: The acceleration in g's.

`float calcGyro(int axis_val)`

**Description**: Converts a raw gyroscope integer value to physical units (degrees per second).
**Returns**: The angular velocity in degrees per second.

`float calcQuat(long quat_val)`

**Description**: Converts a raw Q30 format quaternion value from the DMP into a standard floating-point number.
**Returns**: The floating-point quaternion component.

#### Sensor Configuration

`int setSensors(unsigned char sensors)`: Enables or disables sensors using a bitmask. Available flags are:

* `INV_XYZ_GYRO`: Enables the X, Y, and Z axes of the gyroscope.
* `INV_XYZ_ACCEL`: Enables the X, Y, and Z axes of the accelerometer.
* `INV_XYZ_COMPASS`: Enables the X, Y, and Z axes of the magnetometer (compass). This is only applicable for MPU9250 devices and requires `USE_MPU9250` to be defined.

`int setGyroFSR(unsigned short fsr)`: Sets the gyroscope's full-scale range (250, 500, 1000 or 2000 dps).

`int setAccelFSR(unsigned char fsr)`: Sets the accelerometer's full-scale range (2, 4, 8 or 16 g).

`int setLPF(unsigned short lpf)`: Sets the digital low-pass filter cutoff frequency in Hz.

`int setSampleRate(unsigned short rate)`: Sets the raw sensor sample rate in Hz.

`unsigned short getGyroFSR()`: Gets the current gyro FSR.

`unsigned char getAccelFSR()`: Gets the current accel FSR.

---

## C API Reference (Opaque Pointer)

This C-compatible API provides all the functionality of the C++ class. It is designed for C projects or for creating wrappers in other languages. To use it, include `MPU_DMP_C.h`.

All functions operate on an `MPU_DMP_Handle`, which is an opaque pointer to the underlying C++ object.

### Lifecycle Functions

`MPU_DMP_Handle MPU_DMP_Create()`

* **Description**: Allocates and creates a new `MPU_DMP` instance.
* **Returns**: A handle to the new instance, or `NULL` on failure. This handle must be used in all subsequent API calls.

`void MPU_DMP_Destroy(MPU_DMP_Handle handle)`

* **Description**: Frees all resources associated with an `MPU_DMP` instance.
* **Parameters**:
  * `handle`: The handle returned by `MPU_DMP_Create()`.

### Core Functions

The C functions map directly to their C++ counterparts. The primary difference is that they all take an `MPU_DMP_Handle` as their first argument.

* `int MPU_DMP_begin(MPU_DMP_Handle handle)`
* `int MPU_DMP_dmpBegin(MPU_DMP_Handle handle, unsigned short features, unsigned short fifoRate)`
* `int MPU_DMP_dmpUpdateFifo(MPU_DMP_Handle handle)`
* `int MPU_DMP_computeEulerAngles(MPU_DMP_Handle handle, bool degrees)`

### Data Accessor Functions

Since C cannot directly access the C++ object's members, getter functions are provided to retrieve the sensor data after a call to `MPU_DMP_dmpUpdateFifo()`.

#### Processed Euler Angles

`float MPU_DMP_get_pitch(MPU_DMP_Handle handle)`
`float MPU_DMP_get_roll(MPU_DMP_Handle handle)`
`float MPU_DMP_get_yaw(MPU_DMP_Handle handle)`

#### Raw Quaternion Data

`long MPU_DMP_get_qw(MPU_DMP_Handle handle)`
`long MPU_DMP_get_qx(MPU_DMP_Handle handle)`
`long MPU_DMP_get_qy(MPU_DMP_Handle handle)`
`long MPU_DMP_get_qz(MPU_DMP_Handle handle)`

#### Raw Accelerometer Data

`int MPU_DMP_get_ax(MPU_DMP_Handle handle)`
`int MPU_DMP_get_ay(MPU_DMP_Handle handle)`
`int MPU_DMP_get_az(MPU_DMP_Handle handle)`

#### Raw Gyroscope Data

`int MPU_DMP_get_gx(MPU_DMP_Handle handle)`
`int MPU_DMP_get_gy(MPU_DMP_Handle handle)`
`int MPU_DMP_get_gz(MPU_DMP_Handle handle)`

---

## Example Usage

### C++ Example (Arduino Environment)

This example demonstrates the typical usage pattern for the MPU_DMP class.

```cpp
#include <Wire.h>
#include "MPU_DMP.h"

// Create an MPU_DMP object
MPU_DMP mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize the MPU
    if (mpu.begin() != INV_SUCCESS) {
        Serial.println("Failed to initialize MPU!");
        while (1);
    }

    // Enable the DMP, using 6-axis quaternion fusion
    // Set FIFO rate to 100Hz
    if (mpu.dmpBegin(DMP_FEATURE_6X_LP_QUAT, 100) != INV_SUCCESS) {
        Serial.println("Failed to start DMP!");
        while (1);
    }
    
    Serial.println("MPU and DMP initialized successfully.");
}

void loop() {
    // Check if new DMP data is available
    if (mpu.dmpUpdateFifo() == INV_SUCCESS) {
        // If yes, compute the Euler angles
        mpu.computeEulerAngles();

        // Print the results
        Serial.print("Yaw: ");
        Serial.print(mpu.yaw);
        Serial.print(", Pitch: ");
        Serial.print(mpu.pitch);
        Serial.print(", Roll: ");
        Serial.println(mpu.roll);
    }

    // A small delay is good practice
    delay(10);
}
```

### C Example (TI-RTOS)

This example shows how to use the C API with the opaque pointer.

```c
/* STD header files */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/Board.h>

/* Board */
#include "ti_drivers_config.h"

/* MPU Driver Header */
#include "lib/MPU_DMP_C.h"
#include "lib/util/inv_mpu.h"
#include "lib/util/inv_mpu_dmp_motion_driver.h"

/* Extern declaration for the I2C handle used by the HAL */
extern I2C_Handle i2c_handle;

/*
 * ======== mpuTaskFxn ========
 * Task to initialize and read data from the MPU-6500.
 */
void mpuTaskFxn(UArg arg0, UArg arg1) {
    /* 1. Initialize I2C Driver and set the global handle */
    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c_handle = I2C_open(Board_I2C0, &i2cParams);

    if (i2c_handle == NULL) {
        System_printf("Error Initializing I2C\n");
        System_flush();
        while (1);
    }

    /* 2. Create the MPU_DMP instance */
    MPU_DMP_Handle mpu_handle = MPU_DMP_Create();
    if (!mpu_handle) {
        System_printf("Failed to create MPU_DMP handle.\n");
        System_flush();
        while (1);
    }

    /* 3. Initialize the sensor */
    if (MPU_DMP_begin(mpu_handle) != INV_SUCCESS) {
        System_printf("Failed to initialize MPU sensor.\n");
        System_flush();
        while (1);
    }
    System_printf("MPU initialized.\n");
    System_flush();

    /* 4. Initialize the DMP with 6-axis quaternion feature at 100Hz */
    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_GYRO_CAL;
    if (MPU_DMP_dmpBegin(mpu_handle, dmp_features, 100) != INV_SUCCESS) {
        System_printf("Failed to start DMP.\n");
        System_flush();
        while (1);
    }
    System_printf("DMP started. Reading data...\n");
    System_flush();

    /* 5. Main loop to read and process data */
    while (1) {
        // Try to read a new packet from the FIFO
        if (MPU_DMP_dmpUpdateFifo(mpu_handle) == INV_SUCCESS) {
            // New data is available, compute Euler angles in degrees
            MPU_DMP_computeEulerAngles(mpu_handle, true);

            // Get the calculated angles using accessor functions
            float yaw = MPU_DMP_get_yaw(mpu_handle);
            float pitch = MPU_DMP_get_pitch(mpu_handle);
            float roll = MPU_DMP_get_roll(mpu_handle);

            System_printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", yaw, pitch, roll);
            System_flush();
        }

        // Let other tasks run. Sleep for 10ms.
        // Task_sleep() takes system ticks as an argument.
        Task_sleep(10 * 1000 / Clock_tickPeriod);
    }
}
```
