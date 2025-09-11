# MPU9250 9-DOF IMU Driver API Reference

This document provides a detailed API reference for the `mpu9250_driver`, which interfaces with the TDK InvenSense MPU-9250 9-DOF inertial measurement unit (IMU). The driver is specifically designed for the **Texas Instruments MSP-EXP432E401Y LaunchPad** running the **TI-RTOS** real-time operating system.

The driver is written in C and uses a handle-based interface (`MPU9250_Handle`), which encapsulates the entire state of the sensor instance. It's built to be robust, configurable, and provide a high-level interface for complex tasks like calibration and sensor fusion.

## Key Features

* **Robust Initialization:** Automatically detects MPU9250, MPU9255, and MPU6500 variants.
* **Full Configuration:** Provides easy-to-use structures for configuring sensor ranges, digital low-pass filters (DLPF), and sample rates.
* **Automated Calibration:** Includes functions to calculate biases and variances for the accelerometer, gyroscope, and magnetometer, which are essential for high-quality sensor fusion.
* **Interactive Orientation Setup:** A guided routine helps determine the physical orientation matrix of the sensor, removing the need for hard-coded coordinate transformations.
* **Unified Sensor Fusion Module:** Integrates Complementary, Madgwick, and Mahony filters for stable 6-DoF (Accel + Gyro) and 9-DoF (Accel + Gyro + Mag) orientation tracking.
* **Comprehensive Error Handling:** Returns detailed status codes for easier debugging.

---

## Public Enums and Structures

These are the primary data types used to configure and interact with the driver. Understanding them is key to using the library effectively.

### Status Codes (`MPU9250_Status_e`)

This enum lists all possible return codes for functions that can fail, helping you diagnose issues in your application.

* `MPU9250_OK`: Operation was successful.
* `MPU9250_ERROR_INVALID_HANDLE`: The provided sensor handle was `NULL`.
* `MPU9250_ERROR_MALLOC`: Failed to allocate memory for the handle.
* `MPU9250_ERROR_I2C`: An I2C communication error occurred.
* `MPU9250_ERROR_WHO_AM_I`: The device `WHO_AM_I` value was incorrect, meaning the wrong chip is connected or it's not responding.
* `MPU9250_ERROR_TIMEOUT`: A timeout occurred waiting for a sensor event.
* `MPU9250_ERROR_INVALID_PARAM`: An invalid parameter was passed to a function (e.g., `dt` was zero).
* `MPU9250_ERROR_NOT_SUPPORTED`: The requested feature is not supported by the detected device.

### Sensor Configuration Enums

These enums allow you to configure the physical behavior of the sensor.

* `AccelFSR_e` (Accelerometer Full-Scale Range):
  * `ACCEL_FS_2G`: +/- 2g
  * `ACCEL_FS_4G`: +/- 4g
  * `ACCEL_FS_8G`: +/- 8g
  * `ACCEL_FS_16G`: +/- 16g
* `GyroFSR_e` (Gyroscope Full-Scale Range):
  * `GYRO_FS_250DPS`: +/- 250 degrees per second
  * `GYRO_FS_500DPS`: +/- 500 degrees per second
  * `GYRO_FS_1000DPS`: +/- 1000 degrees per second
  * `GYRO_FS_2000DPS`: +/- 2000 degrees per second
* `AccelDLPF_e` (Accelerometer Digital Low-Pass Filter):
  * `ACCEL_DLPF_218HZ`
  * `ACCEL_DLPF_99HZ`
  * `ACCEL_DLPF_41HZ`
  * `ACCEL_DLPF_20HZ`
  * `ACCEL_DLPF_10HZ`
  * `ACCEL_DLPF_5HZ`
* `GyroDLPF_e` (Gyroscope Digital Low-Pass Filter):
  * `GYRO_DLPF_184HZ`
  * `GYRO_DLPF_92HZ`
  * `GYRO_DLPF_41HZ`
  * `GYRO_DLPF_20HZ`
  * `GYRO_DLPF_10HZ`
  * `GYRO_DLPF_5HZ`
* `MagMode_e` (Magnetometer Mode):
  * `MAG_MODE_8HZ`: 8 Hz continuous measurement.
  * `MAG_MODE_100HZ`: 100 Hz continuous measurement.
* `MagOutput_e` (Magnetometer Resolution):
  * `MAG_OUT_14_BIT`: 14-bit output resolution.
  * `MAG_OUT_16_BIT`: 16-bit output resolution.

### Sensor Fusion Enums

These enums are used to control the sensor fusion module.

* `FusionAlgorithm_e`:
  * `FUSION_ALGORITHM_COMPLEMENTARY`: Simple Complementary Filter.
  * `FUSION_ALGORITHM_MADGWICK`: Madgwick Gradient Descent AHRS.
  * `FUSION_ALGORITHM_MAHONY`: Mahony PI Controller AHRS.
* `FusionDOF_e` (Degrees of Freedom):
  * `FUSION_DOF_6`: Use Accelerometer and Gyroscope only.
  * `FUSION_DOF_9`: Use Accelerometer, Gyroscope, and Magnetometer.

### Core Data Structures

* `MPU9250_Config`: This structure holds all the initial settings for the sensor. You populate it and pass it to `MPU9250_init`.
  * `uint8_t i2cAddress`: The I2C address of the sensor (0x68 or 0x69).
  * `AccelFSR_e accelFSR`: Accelerometer full-scale range.
  * `GyroFSR_e gyroFSR`: Gyroscope full-scale range.
  * `uint8_t sampleRateDivider`: Divides the internal 1kHz sample rate. `Sample Rate = 1kHz / (1 + divider)`.
  * `AccelDLPF_e accelDLPF`: Accelerometer Digital Low-Pass Filter setting.
  * `GyroDLPF_e gyroDLPF`: Gyroscope Digital Low-Pass Filter setting.
  * `MagMode_e magMode`: Magnetometer continuous measurement mode.
  * `MagOutput_e magOutput`: Magnetometer output bit resolution.
* `MPU9250_Handle`: This is the main object for your sensor instance. It's created by `MPU9250_init` and holds the complete state of the driver and sensor.
  * **TI-RTOS Communication Handles**
    * `I2C_Handle i2cHandle`: Handle to the I2C peripheral from TI-RTOS.
    * `I2C_Transaction *i2cTransaction`: Pointer to a user-provided I2C transaction struct.
    * `uint8_t i2cAddress`: The I2C address of the sensor.
  * **Sensor State & Data**
    * `bool hasMag`: `true` if an AK8963 magnetometer is detected.
    * `MPU9250_Vector3D accelerometer`: Latest accelerometer reading in m/s².
    * `MPU9250_Vector3D gyroscope`: Latest gyroscope reading in rad/s.
    * `MPU9250_Vector3D magnetometer`: Latest magnetometer reading in µT.
    * `float temperature`: Latest temperature reading in Celsius.
    * `float sampleRate`: The configured sample rate in Hz.
  * **Calibration Data**
    * `MPU9250_Vector3D accelBias`, `gyroBias`, `magBias`: Biases (offsets) calculated by `MPU9250_calibrate` and automatically subtracted from raw readings.
    * `MPU9250_Vector3D accelVariance`, `gyroVariance`, `magVariance`: Measurement variances, useful for advanced filters like an EKF.
  * **Internal Scale Factors**
    * `float accelScale`, `gyroScale`, `magScale`: Conversion factors from raw sensor values to physical units.
    * `MPU9250_Vector3D magAdjust`: Magnetometer sensitivity adjustments read from the sensor's internal ROM.
  * **Sensor Fusion State**
    * `MPU9250_FusionState fusion`: A nested struct holding the state and results of the chosen fusion algorithm.

---

## Sensor Fusion Algorithms Explained

A raw IMU gives you noisy, drifting data. Sensor fusion algorithms combine the data from multiple sensors to produce a stable and accurate estimate of the device's orientation. This driver includes three popular algorithms.

### Complementary Filter

This is the simplest and most intuitive of the filters. It works in the frequency domain.

* **Concept**: It combines the gyroscope and accelerometer data by trusting each one in the domain where it performs best.
* **Gyroscopes** are good at tracking *fast* changes in orientation (high-frequency), but they drift over time.
* **Accelerometers** can determine the direction of gravity (and thus roll/pitch) when the device is still or moving slowly, making them a stable reference for *slow* changes (low-frequency).
* **How it Works**: The filter is essentially a weighted average. The new angle is calculated as:
  `angle = α * (previous_angle + gyro_rate * dt) + (1 - α) * accel_angle`
* The `α` (alpha) parameter is a filter coefficient (e.g., 0.98). A higher value means you trust the gyroscope more, making the filter responsive but more prone to drift. A lower value trusts the accelerometer more, making it stable but sluggish.
* **Pros**: Computationally very cheap and easy to understand.
* **Cons**: Less accurate than other methods, especially during complex movements. Yaw (heading) can only be determined with a magnetometer and is also subject to drift if only using the gyro.

### Madgwick AHRS Filter

This is a much more sophisticated algorithm based on quaternions and gradient descent optimization.

* **Concept**: It uses the gyroscope to predict the orientation and then uses the accelerometer and magnetometer as a reference to correct for drift.
* **How it Works**:

1. The gyroscope data is integrated to predict the new orientation quaternion.
2. An "objective function" is created using the accelerometer and magnetometer data. This function represents the error between the orientation predicted by the gyro and the orientation measured by the reference sensors.
3. A mathematical technique called **gradient descent** is used to find the direction in which the orientation quaternion needs to be adjusted to minimize this error.
4. The quaternion is adjusted in that direction by a small amount, controlled by the gain parameter `β` (beta). `β` represents the gyroscope measurement error; a higher value corrects the orientation more aggressively.

* **Pros**: Very accurate and robust across a wide range of motions. It's considered one of the gold standards for IMU orientation filtering.
* **Cons**: More computationally expensive than the Complementary or Mahony filters.

### Mahony AHRS Filter

The Mahony filter is another advanced quaternion-based filter that is computationally more efficient than the Madgwick filter.

* **Concept**: It works like a **Proportional-Integral (PI) controller**, a common concept in control theory. The "error" is the difference between the direction of gravity measured by the accelerometer and the direction of gravity estimated by the current orientation.
* **How it Works**:

1. The error between the measured and estimated reference vectors (gravity and magnetic north) is calculated.
2. This error is fed into a PI controller.
   * The **Proportional gain (`Kp`)** makes an immediate correction to the orientation based on the current error. It's like pushing the estimate back towards the correct orientation.
   * The **Integral gain (`Ki`)** accumulates the error over time. This term is used to estimate and cancel out the gyroscope's bias drift, which is the primary source of long-term error.
3. The output of the PI controller is used to correct the angular velocity from the gyroscope before it's integrated.

* **Pros**: Almost as accurate as the Madgwick filter but significantly less computationally intensive, making it ideal for microcontrollers with limited processing power.
* **Cons**: Can be slightly less responsive than the Madgwick filter under very high-speed, complex motion.

---

## API Function Reference

All functions are exposed in `mpu9250_driver.h`.

### Lifecycle & Initialization

`void MPU9250_getDefaultConfig(MPU9250_Config *config)`

* **Description**: Fills a configuration structure with default, recommended settings for general-purpose use.
* **Parameters**:
* `[out] config`: A pointer to the configuration structure to be populated.

`MPU9250_Status_e MPU9250_init(MPU9250_Handle **sensor_p, I2C_Handle i2c, I2C_Transaction *transaction, const MPU9250_Config *config)`

* **Description**: Initializes the MPU9250 sensor. This is the first function that must be called. It allocates memory for the sensor handle, resets the device, verifies its identity (`WHO_AM_I`), and applies the settings from the configuration structure.
* **Parameters**:
* `[out] sensor_p`: A pointer to the handle pointer (`MPU9250_Handle*`). The driver will allocate memory and store the address of the new handle here.
* `[in] i2c`: A handle to an initialized TI-RTOS I2C peripheral.
* `[in] transaction`: A pointer to a user-provided `I2C_Transaction` structure. The driver will use this for all I2C communication.
* `[in] config`: A pointer to a configuration structure with the desired settings.
* **Returns**: `MPU9250_OK` on success, or an error code on failure.

`void MPU9250_deinit(MPU9250_Handle *sensor)`

* **Description**: Deinitializes the sensor and frees the memory allocated for the handle.
* **Parameters**:
* `[in] sensor`: The handle to deinitialize.

### Data Acquisition

`MPU9250_Status_e MPU9250_readSensors(MPU9250_Handle *sensor)`

* **Description**: Performs a burst-read to fetch the latest data from the accelerometer, gyroscope, temperature sensor, and (if present) the magnetometer. It applies calibration biases and scaling factors to convert the raw data into standard physical units (m/s², rad/s, µT, °C). The results are stored within the `sensor` handle (e.g., `sensor->accelerometer`, `sensor->gyroscope`).
* **Parameters**:
* `[in,out] sensor`: Pointer to the initialized sensor handle.
* **Returns**: `MPU9250_OK` on success, or an error code on failure.

### Calibration & Setup

`MPU9250_Status_e MPU9250_calibrate(MPU9250_Handle *sensor, uint16_t numSamples)`

* **Description**: Performs a static calibration. It averages a number of samples to calculate the bias (offset) and variance for each sensor axis. These values are stored in the handle and automatically subtracted from future readings.
* **Precondition**: The device must be kept perfectly still and level during the calibration process.
* **Parameters**:
* `[in,out] sensor`: Pointer to the initialized sensor handle.
* `[in] numSamples`: The number of samples to collect and average (e.g., 1000). More samples yield better results but take longer.
* **Returns**: `MPU9250_OK` on success, or an error code on failure.

`MPU9250_Status_e MPU9250_determineOrientationAxes(MPU9250_Handle *sensor, MPU9250_Matrix3x3 *matrix, ReferenceFrame_e frame)`

* **Description**: Runs an interactive, two-step console routine to determine the sensor's physical orientation relative to a **user-selected reference frame** (`NED` or `ENU`). It populates a rotation matrix that can be used with `MPU9250_applyRotation` to transform sensor readings from the body frame to the selected world frame. The interactive process guides the user to first establish the vertical axis by measuring gravity and then find the forward (North) axis by performing a specific motion.
* **Parameters**:
* `[in] sensor`: Pointer to the initialized and calibrated sensor handle.
* `[out] matrix`: A pointer to a 3x3 matrix where the resulting orientation matrix will be stored.
* `[in] frame`: The desired reference frame for calibration. This must be one of the values from the `ReferenceFrame_e` enum (`FRAME_NED` or `FRAME_ENU`).
* **Returns**: `MPU9250_OK` on success, or an error code on failure.

---

#### **Reference Frames Explained**

The `frame` parameter allows you to align the sensor's data with one of two common coordinate systems used in navigation and robotics.

* **NED (North-East-Down)**

  This is a local tangent plane frame commonly used in **aerospace and aviation** applications.

  * **X-axis:** Points to true **North**.
  * **Y-axis:** Points to true **East**.
  * **Z-axis:** Points **Down**, towards the center of the Earth.
* **ENU (East-North-Up)**

  This frame is widely used in **geodetic, surveying, and ground-based robotics** applications.

  * **X-axis:** Points to true **East**.
  * **Y-axis:** Points to true **North**.
  * **Z-axis:** Points **Up**, away from the center of the Earth.

---

`void MPU9250_applyRotation(const MPU9250_Matrix3x3 *matrix, MPU9250_Vector3D *vector)`

* **Description**: A utility function to apply a coordinate transformation to a 3D vector using a rotation matrix (`V_rotated = R * V_original`). This is useful for rotating sensor data from the body frame to the world frame before sensor fusion.
* **Parameters**:
* `[in] matrix`: Pointer to the 3x3 rotation matrix.
* `[in,out] vector`: Pointer to the vector to be rotated. The operation is done in-place.

### Sensor Fusion

`MPU9250_Status_e MPU9250_SensorFusionInit(MPU9250_Handle *sensor, FusionAlgorithm_e algorithm, const void *params)`

* **Description**: Initializes the sensor fusion module. This sets the desired algorithm and its associated parameters (e.g., filter gain).
* **Parameters**:
* `[in,out] sensor`: Pointer to the sensor handle.
* `[in] algorithm`: The fusion algorithm to use (e.g., `FUSION_ALGORITHM_MADGWICK`).
* `[in] params`: A void pointer to a parameter structure corresponding to the chosen algorithm (e.g., a `MadgwickFilterParams` struct).
* **Returns**: `MPU9250_OK` on success, or an error code.

`MPU9250_Status_e MPU9250_SensorFusionUpdate(MPU9250_Handle *sensor, FusionDOF_e dof, float dt)`

* **Description**: Runs one iteration of the configured fusion algorithm. It uses the latest sensor data stored in the handle to update the orientation estimate. The result is stored internally in `sensor->fusion.orientation`.
* **Preconditions**:

1. `MPU9250_readSensors()` should be called immediately before this function to get fresh data.
2. For best results, sensor data should be transformed to the world frame via `MPU9250_applyRotation()` before calling this function.

* **Parameters**:
* `[in,out] sensor`: Pointer to the sensor handle.
* `[in] dof`: The degrees of freedom to use for the update (`FUSION_DOF_6` for Accel+Gyro, `FUSION_DOF_9` for Accel+Gyro+Mag).
* `[in] dt`: The time elapsed since the last update, in seconds. This is critical for accurate integration.
* **Returns**: `MPU9250_OK` on success, or an error code.

### Data Retrieval

`MPU9250_Status_e MPU9250_getOrientation(MPU9250_Handle *sensor, MPU9250_Quaternion *q)`

* **Description**: Gets the current orientation calculated by the fusion filter as a quaternion.
* **Parameters**:
* `[in] sensor`: Pointer to the sensor handle.
* `[out] q`: Pointer to a quaternion structure where the result will be stored.
* **Returns**: `MPU9250_OK` on success, or an error code.

`MPU9250_Status_e MPU9250_getEulerAngles(MPU9250_Handle *sensor, MPU9250_EulerAngles *angles)`

* **Description**: Gets the current orientation as Euler angles (roll, pitch, yaw). It converts the internal orientation quaternion to Euler angles in  **radians**.
* **Parameters**:
* `[in] sensor`: Pointer to the sensor handle.
* `[out] angles`: Pointer to an `MPU9250_EulerAngles` structure where the result will be stored.
* **Returns**: `MPU9250_OK` on success, or an error code.

---

## Example Usage (TI-RTOS)

This example demonstrates the typical usage pattern for the MPU9250 driver within a TI-RTOS task.

```c
#include <stdint.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h" // Generated by SysConfig

#include "mpu9250_driver.h"

#define SENSOR_TASK_PRIORITY   3
#define SENSOR_TASK_STACK_SIZE 2048
#define SENSOR_UPDATE_PERIOD_MS 10 // For a 100 Hz update rate

void sensorTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cHandle;
    I2C_Params i2cParams;
    I2C_Transaction i2cTransaction;

    MPU9250_Handle *sensor_handle = NULL;
    MPU9250_Config sensor_config;
    MPU9250_EulerAngles euler_angles;

    /* 1. Initialize I2C Driver */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2cHandle == NULL) {
        System_printf("Error Initializing I2C\n");
        System_flush();
        while (1);
    }

    // Configure the transaction struct. This will be used by the driver.
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;

    /* 2. Initialize the MPU9250 Driver */
    MPU9250_getDefaultConfig(&sensor_config); // Start with defaults
    sensor_config.accelFSR = ACCEL_FS_4G;
    sensor_config.gyroFSR = GYRO_FS_500DPS;
    sensor_config.sampleRateDivider = 9; // Sets sample rate to 1kHz / (1+9) = 100Hz

    if (MPU9250_init(&sensor_handle, i2cHandle, &i2cTransaction, &sensor_config) != MPU9250_OK) {
        System_printf("Failed to initialize MPU9250 sensor.\n");
        while (1);
    }
    System_printf("MPU9250 Initialized.\n");

    /* 3. Calibrate the sensor */
    System_printf("Calibrating... Keep the sensor still.\n");
    if (MPU9250_calibrate(sensor_handle, 1000) != MPU9250_OK) {
        System_printf("Calibration failed.\n");
        while(1);
    }
    System_printf("Calibration complete.\n");

    /* 4. Initialize the Sensor Fusion Module (using Madgwick) */
    MadgwickFilterParams fusion_params = {.beta = 0.1f};
    if (MPU9250_SensorFusionInit(sensor_handle, FUSION_ALGORITHM_MADGWICK, &fusion_params) != MPU9250_OK) {
        System_printf("Failed to init sensor fusion.\n");
        while(1);
    }
    System_printf("Sensor Fusion Initialized. Reading data...\n");
    System_flush();

    float dt = (float)SENSOR_UPDATE_PERIOD_MS / 1000.0f;

    /* 5. Main loop to read and process data */
    while (1) {
        // Delay to maintain a consistent update rate
        Task_sleep(SENSOR_UPDATE_PERIOD_MS * 1000 / Clock_tickPeriod);

        // Read the latest sensor data
        if (MPU9250_readSensors(sensor_handle) == MPU9250_OK) {

            // Update the fusion algorithm with the new data
            MPU9250_SensorFusionUpdate(sensor_handle, FUSION_DOF_9, dt);

            // Get the resulting orientation in Euler angles
            MPU9250_getEulerAngles(sensor_handle, &euler_angles);

            // Convert radians to degrees for printing
            float roll_deg = euler_angles.roll * 180.0f / M_PI;
            float pitch_deg = euler_angles.pitch * 180.0f / M_PI;
            float yaw_deg = euler_angles.yaw * 180.0f / M_PI;

            System_printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);
            System_flush();
        }
    }
}
```
