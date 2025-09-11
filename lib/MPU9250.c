// ================================================================================
// 
// File Name           : MPU9250.c
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Source code file for MPU9250 IMU sensor driver
//      defines constants, register maps, function prototypes, and hardware drivers
//      for interacting via I2C and a GPIO on TI-RTOS
// 
// Author              : Adrià Babiano Novella
// Create Date         : 2024-12-01
// Revision            : v2.0
// ================================================================================

/**
 * @file MPU9250.c
 * @brief Implementation file for the MPU9250 IMU sensor driver.
 *
 * Contains functions for initializing, configuring, reading from, and writing to
 * the MPU9250 sensor using I2C on a TI-RTOS platform. Also includes data
 * conversion utilities.
 */

#include "MPU9250.h" // Include the header file for this module


// --- Helper Functions ---
/**
 * @brief Controls the power supply to the MPU9250 via GPIO.
 * @param pwr Boolean indicating desired power state (true = ON, false = OFF). [in]
 */
static void _PWRIMU(MPU9250_Handle *sensor, bool pwr) {
    // Write the GPIO pin state: 1 for true (ON), 0 for false (OFF)
    GPIO_write(sensor->pwrGPIO, pwr ? 1 : 0);
}

/**
 * @brief  Writes a single byte to a specific MPU6500 register.
 * @param  sensor Pointer to the sensor handle.
 * @param  reg The 8-bit address of the register to write to.
 * @param  val The 8-bit data byte to write.
 * @return True on success, false on I2C transfer failure.
 */
static bool _writeReg(MPU9250_Handle *sensor, uint8_t reg, uint8_t val) {
    uint8_t txBuffer[2];
    txBuffer[0] = reg;
    txBuffer[1] = val;
    sensor->i2cTransaction->writeBuf = txBuffer;
    sensor->i2cTransaction->writeCount = 2;
    sensor->i2cTransaction->readBuf = NULL;
    sensor->i2cTransaction->readCount = 0;
    return I2C_transfer(sensor->i2cHandle, sensor->i2cTransaction);
}

/**
 * @brief  Reads one or more bytes from a specific MPU6500 register.
 * @param  sensor Pointer to the sensor handle.
 * @param  reg The 8-bit address of the starting register to read from.
 * @param  data Pointer to a buffer where the read data will be stored.
 * @param  len The number of bytes to read.
 * @return True on success, false on I2C transfer failure.
 */
static bool _readRegs(MPU9250_Handle *sensor, uint8_t reg, uint8_t *data, uint16_t len) {
    sensor->i2cTransaction->writeBuf = &reg;
    sensor->i2cTransaction->writeCount = 1;
    sensor->i2cTransaction->readBuf = data;
    sensor->i2cTransaction->readCount = len;
    return I2C_transfer(sensor->i2cHandle, sensor->i2cTransaction);
}

/**
 * @brief  Writes a single byte to a magnetometer (AK8963) register.
 * @details Configures the MPU6500's I2C master interface to perform a write
 * transaction to the external magnetometer.
 * @param  sensor Pointer to the sensor handle.
 * @param  mag_reg The 8-bit address of the AK8963 register to write to.
 * @param  val The 8-bit data byte to write.
 */
static void _magWriteReg(MPU9250_Handle *sensor, uint8_t mag_reg, uint8_t val) {
    _writeReg(sensor, MPU6500_I2C_SLV0_ADDR_ADDR, AK8963_I2C_ADDR);
    _writeReg(sensor, MPU6500_I2C_SLV0_REG_ADDR, mag_reg);
    _writeReg(sensor, MPU6500_I2C_SLV0_DO_ADDR, val);
    _writeReg(sensor, MPU6500_I2C_SLV0_CTRL_ADDR, MPU6500_I2C_SLV_EN_BIT | 1);
    Task_sleep(10);
}

/**
 * @brief  Reads one or more bytes from a magnetometer (AK8963) register.
 * @details Configures the MPU6500's I2C master interface to perform a read
 * transaction from the external magnetometer and retrieve the data
 * from the EXT_SENS_DATA registers.
 * @param  sensor Pointer to the sensor handle.
 * @param  mag_reg The 8-bit address of the starting AK8963 register to read from.
 * @param  data Pointer to a buffer where the read data will be stored.
 * @param  len The number of bytes to read.
 * @return True on success, false on I2C transfer failure.
 */
static bool _magReadRegs(MPU9250_Handle *sensor, uint8_t mag_reg, uint8_t *data, uint16_t len) {
    _writeReg(sensor, MPU6500_I2C_SLV0_ADDR_ADDR, AK8963_I2C_ADDR | MPU6500_I2C_SLV_RNW_BIT);
    _writeReg(sensor, MPU6500_I2C_SLV0_REG_ADDR, mag_reg);
    _writeReg(sensor, MPU6500_I2C_SLV0_CTRL_ADDR, MPU6500_I2C_SLV_EN_BIT | len);
    Task_sleep(10 * len);
    return _readRegs(sensor, MPU6500_EXT_SENS_DATA_00_ADDR, data, len);
}

/**
 * @brief  Calculates the cross product of two 3D vectors.
 * @param  a Pointer to the first vector.
 * @param  b Pointer to the second vector.
 * @return The resulting cross product vector.
 */
static MPU9250_Vector3D _vec_cross(const MPU9250_Vector3D *a, const MPU9250_Vector3D *b) {
    MPU9250_Vector3D result;
    result.x = a->y * b->z - a->z * b->y;
    result.y = a->z * b->x - a->x * b->z;
    result.z = a->x * b->y - a->y * b->x;
    return result;
}

/**
 * @brief  Calculates the fast inverse square root of a number.
 * @details An implementation of the famous Q_rsqrt algorithm, used for efficient
 * vector normalization in the Madgwick and Mahony filters.
 * @param  x The number to calculate the inverse square root of.
 * @return The inverse square root of x.
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief Updates the orientation using the Complementary Filter.
 * @param sensor Pointer to the MPU9250 handle.
 * @param dof The degrees of freedom to use (FUSION_DOF_6 or FUSION_DOF_9).
 * @param dt The time delta in seconds since the last update.
 */
static void _updateComplementaryFilter(MPU9250_Handle *sensor, FusionDOF_e dof, float dt) {
    MPU9250_FusionState *fusion = &sensor->fusion;
    float alpha = fusion->params.comp.alpha;
    float roll_accel, pitch_accel;
    float mx, my, mz, roll, pitch;
    float mag_x_comp, mag_y_comp, yaw_mag;
    float cy, sy, cp, sp, cr, sr;

    /* --- Step 1: Calculate Roll and Pitch from Accelerometer & Gyro --- */
    roll_accel = atan2f(sensor->accelerometer.y, sensor->accelerometer.z);
    pitch_accel = atan2f(-sensor->accelerometer.x, sqrtf(sensor->accelerometer.y * sensor->accelerometer.y + sensor->accelerometer.z * sensor->accelerometer.z));

    fusion->euler.roll = alpha * (fusion->euler.roll + sensor->gyroscope.x * dt) + (1.0f - alpha) * roll_accel;
    fusion->euler.pitch = alpha * (fusion->euler.pitch + sensor->gyroscope.y * dt) + (1.0f - alpha) * pitch_accel;

    /* --- Step 2: Calculate Yaw --- */
    if (dof == FUSION_DOF_9 && sensor->hasMag) {
        /* --- 9-DoF: Use Magnetometer with Tilt Compensation --- */
        mx = sensor->magnetometer.x;
        my = sensor->magnetometer.y;
        mz = sensor->magnetometer.z;
        roll = fusion->euler.roll;
        pitch = fusion->euler.pitch;

        mag_x_comp = mx * cosf(pitch) + my * sinf(roll) * sinf(pitch) - mz * cosf(roll) * sinf(pitch);
        mag_y_comp = my * cosf(roll) + mz * sinf(roll);
        yaw_mag = atan2f(-mag_y_comp, mag_x_comp);

        fusion->euler.yaw = alpha * (fusion->euler.yaw + sensor->gyroscope.z * dt) + (1.0f - alpha) * yaw_mag;
    } else {
        /* --- 6-DoF: Simple Gyroscope Integration for Yaw --- */
        fusion->euler.yaw += sensor->gyroscope.z * dt;
    }

    /* --- Step 3: Convert Final Euler Angles to Quaternion --- */
    cy = cosf(fusion->euler.yaw * 0.5f);
    sy = sinf(fusion->euler.yaw * 0.5f);
    cp = cosf(fusion->euler.pitch * 0.5f);
    sp = sinf(fusion->euler.pitch * 0.5f);
    cr = cosf(fusion->euler.roll * 0.5f);
    sr = sinf(fusion->euler.roll * 0.5f);

    fusion->orientation.w = cr * cp * cy + sr * sp * sy;
    fusion->orientation.x = sr * cp * cy - cr * sp * sy;
    fusion->orientation.y = cr * sp * cy + sr * cp * sy;
    fusion->orientation.z = cr * cp * sy - sr * sp * cy;
}

/**
 * @brief Updates the orientation quaternion using the Madgwick AHRS algorithm.
 * @param sensor Pointer to the MPU9250 handle.
 * @param dof The degrees of freedom to use (FUSION_DOF_6 or FUSION_DOF_9).
 * @param dt The time delta in seconds since the last update.
 */
static void _updateMadgwickFilter(MPU9250_Handle *sensor, FusionDOF_e dof, float dt) {
    /* (Implementation from previous response, C99 compliant) */
    MPU9250_FusionState *fusion = &sensor->fusion;
    MPU9250_Quaternion *q = &fusion->orientation;
    float beta = fusion->params.madgwick.beta;
    float ax = sensor->accelerometer.x, ay = sensor->accelerometer.y, az = sensor->accelerometer.z;
    float gx = sensor->gyroscope.x, gy = sensor->gyroscope.y, gz = sensor->gyroscope.z;
    float mx = sensor->magnetometer.x, my = sensor->magnetometer.y, mz = sensor->magnetometer.z;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    bool useMag = (dof == FUSION_DOF_9) && sensor->hasMag && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

        if (useMag) {
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
            _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz; _2q1mx = 2.0f * q1 * mx;
            hx = mx * (q0q0 + q1q1 - q2q2 - q3q3) + my * (2.0f * (q1q2 - q0q3)) + mz * (2.0f * (q1q3 + q0q2));
            hy = mx * (2.0f * (q1q2 + q0q3)) + my * (q0q0 - q1q1 + q2q2 - q3q3) + mz * (2.0f * (q2q3 - q0q1));
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = mx * (2.0f * (q1q3 - q0q2)) + my * (2.0f * (q2q3 + q0q1)) + mz * (q0q0 - q1q1 - q2q2 + q3q3);
            _4bx = 2.0f * _2bx; _4bz = 2.0f * _2bz;
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        } else {
            s0 = -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2.0f * (q0q1 + q2q3) - ay);
            s1 = _2q3 * (2.0f * (q1q3 - q0q2) - ax) + _2q0 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q1 * (1.0f - 2.0f * (q1q1 + q2q2) - az);
            s2 = -_2q0 * (2.0f * (q1q3 - q0q2) - ax) + _2q3 * (2.0f * (q0q1 + q2q3) - ay) - 4.0f * q2 * (1.0f - 2.0f * (q1q1 + q2q2) - az);
            s3 = _2q1 * (2.0f * (q1q3 - q0q2) - ax) + _2q2 * (2.0f * (q0q1 + q2q3) - ay);
        }
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
        qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->w = q0 * recipNorm; q->x = q1 * recipNorm; q->y = q2 * recipNorm; q->z = q3 * recipNorm;
}

/**
 * @brief Updates the orientation quaternion using the Mahony AHRS algorithm.
 * @param sensor Pointer to the MPU9250 handle.
 * @param dof The degrees of freedom to use (FUSION_DOF_6 or FUSION_DOF_9).
 * @param dt The time delta in seconds since the last update.
 */
static void _updateMahonyFilter(MPU9250_Handle *sensor, FusionDOF_e dof, float dt) {
    /* (Implementation from previous response, C99 compliant) */
    MPU9250_FusionState *fusion = &sensor->fusion;
    float Kp = fusion->params.mahony.Kp;
    float Ki = fusion->params.mahony.Ki;
    MPU9250_Quaternion *q = &fusion->orientation;
    float ax = sensor->accelerometer.x, ay = sensor->accelerometer.y, az = sensor->accelerometer.z;
    float gx = sensor->gyroscope.x, gy = sensor->gyroscope.y, gz = sensor->gyroscope.z;
    float mx = sensor->magnetometer.x, my = sensor->magnetometer.y, mz = sensor->magnetometer.z;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfex, halfey, halfez;
    float qa, qb, qc;
    bool useMag = (dof == FUSION_DOF_9) && sensor->hasMag && !((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        if (useMag) {
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
            q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
            q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
            q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
            halfvx = q1q3 - q0q2; halfvy = q0q1 + q2q3; halfvz = q0q0 - 0.5f + q3q3;
            float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        } else {
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
        }

        if (Ki > 0.0f) {
            fusion->mahonyIntegralError.x += Ki * halfex * dt;
            fusion->mahonyIntegralError.y += Ki * halfey * dt;
            fusion->mahonyIntegralError.z += Ki * halfez * dt;
            gx += fusion->mahonyIntegralError.x;
            gy += fusion->mahonyIntegralError.y;
            gz += fusion->mahonyIntegralError.z;
        } else {
            fusion->mahonyIntegralError = (MPU9250_Vector3D){0};
        }
        gx += Kp * halfex; gy += Kp * halfey; gz += Kp * halfez;
    }

    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->w = q0 * recipNorm; q->x = q1 * recipNorm; q->y = q2 * recipNorm; q->z = q3 * recipNorm;
}

// --- Public Functions ---

void MPU9250_getDefaultConfig(MPU9250_Config *config) {
    if (!config) return;
    config->i2cAddress = 0x68;
    config->accelFSR = ACCEL_FS_2G;
    config->gyroFSR = GYRO_FS_500DPS;
    config->sampleRateDivider = 4;
    config->accelDLPF = ACCEL_DLPF_41HZ;
    config->gyroDLPF = GYRO_DLPF_41HZ;
    config->magMode = MAG_MODE_100HZ;
    config->magOutput = MAG_OUT_16_BIT;
    config->interrupts.pinConfig.level = ACTIVE_HIGH;
    config->interrupts.pinConfig.mode = PUSH_PULL;
    config->interrupts.pinConfig.latching = true;
    config->interrupts.enabledSources = INT_ENABLE_RAW_RDY;
    config->fifo.mode = FIFO_MODE_STOP_ON_FULL;
    config->fifo.enabledSources = FIFO_EN_ALL_SENSORS;
    config->i2cMasterClock = I2C_MASTER_CLK_348KHZ;
    config->enableFsync = false;
}

MPU9250_Status_e MPU9250_init(MPU9250_Handle **sensor_p, const I2C_Handle i2c, const I2C_Transaction *transaction, const MPU9250_Config *config, const uint8_t pwrGPIO) {
    MPU9250_Handle *sensor;
    uint8_t whoami = 0;
    float accel_fsr_val, gyro_fsr_val;
    uint8_t asa[3];
    uint8_t cntl1;

    if (!sensor_p || !i2c || !transaction || !config) {
        return MPU9250_ERROR_INVALID_PARAM;
    }

    sensor = (MPU9250_Handle*)malloc(sizeof(MPU9250_Handle));
    if (!sensor) return MPU9250_ERROR_MALLOC;
    memset(sensor, 0, sizeof(MPU9250_Handle));
    *sensor_p = sensor;

    sensor->i2cHandle = i2c;
    sensor->i2cTransaction = transaction;
    sensor->i2cAddress = config->i2cAddress;
    sensor->i2cTransaction->slaveAddress = config->i2cAddress;
    sensor->sampleRate = 1000.0f / (1.0f + config->sampleRateDivider);
    sensor->pwrGPIO = pwrGPIO;

    if (!_writeReg(sensor, MPU6500_PWR_MGMT_1_ADDR, MPU6500_PWR_MGMT_1_H_RESET_BIT)) return MPU9250_ERROR_I2C;
    Task_sleep(100);

    if (!_writeReg(sensor, MPU6500_PWR_MGMT_1_ADDR, MPU6500_CLKSEL_AUTO_SELECT)) return MPU9250_ERROR_I2C;
    Task_sleep(10);

    if (!_readRegs(sensor, MPU6500_WHO_AM_I_ADDR, &whoami, 1)) return MPU9250_ERROR_I2C;
    switch(whoami) {
        case MPU9250_WHO_AM_I:
            sensor->hasMag = true;
            break;
        case MPU9255_WHO_AM_I:
            sensor->hasMag = true;
            break;
        case MPU6500_WHO_AM_I:
            sensor->hasMag = false;
            break;
        default:
            MPU9250_deinit(sensor);
            return MPU9250_ERROR_WHO_AM_I;
    }

    if (!_writeReg(sensor, MPU6500_CONFIG_ADDR, config->gyroDLPF)) return MPU9250_ERROR_I2C;
    if (!_writeReg(sensor, MPU6500_GYRO_CONFIG_ADDR, config->gyroFSR)) return MPU9250_ERROR_I2C;
    if (!_writeReg(sensor, MPU6500_ACCEL_CONFIG_ADDR, config->accelFSR)) return MPU9250_ERROR_I2C;
    if (!_writeReg(sensor, MPU6500_ACCEL_CONFIG_2_ADDR, config->accelDLPF)) return MPU9250_ERROR_I2C;
    if (!_writeReg(sensor, MPU6500_SMPLRT_DIV_ADDR, config->sampleRateDivider)) return MPU9250_ERROR_I2C;

    accel_fsr_val = 2.0f * (1 << config->accelFSR);
    gyro_fsr_val = 250.0f * (1 << config->gyroFSR);
    sensor->accelScale = (accel_fsr_val * G_MPS2) / 32768.0f;
    sensor->gyroScale = (gyro_fsr_val * DEG_TO_RAD) / 32768.0f;

    if (sensor->hasMag) {
        if (!_writeReg(sensor, MPU6500_USER_CTRL_ADDR, MPU6500_USER_CTRL_I2C_MST_EN_BIT)) return MPU9250_ERROR_I2C;
        if (!_writeReg(sensor, MPU6500_I2C_MST_CTRL_ADDR, 0x0D)) return MPU9250_ERROR_I2C;

        if (!_magReadRegs(sensor, AK8963_WIA_ADDR, &whoami, 1) || whoami != AK8963_WHO_AM_I_VALUE) {
            sensor->hasMag = false;
        } else {
            _magWriteReg(sensor, AK8963_CNTL1_ADDR, AK8963_MODE_FUSE_ROM);
            if (!_magReadRegs(sensor, AK8963_ASAX_ADDR, asa, 3)) return MPU9250_ERROR_I2C;
            sensor->magAdjust.x = (float)(asa[0] - 128) / 256.0f + 1.0f;
            sensor->magAdjust.y = (float)(asa[1] - 128) / 256.0f + 1.0f;
            sensor->magAdjust.z = (float)(asa[2] - 128) / 256.0f + 1.0f;
            _magWriteReg(sensor, AK8963_CNTL1_ADDR, AK8963_MODE_POWER_DOWN);
            cntl1 = (config->magOutput << 4) | config->magMode;
            _magWriteReg(sensor, AK8963_CNTL1_ADDR, cntl1);
            sensor->magScale = (config->magOutput == MAG_OUT_16_BIT) ? (4912.0f / 32760.0f) : (4912.0f / 8190.0f);
        }
    }
    return MPU9250_OK;

    System_printf("MPU9250 Initialization complete.\n");
    System_flush();
}

void MPU9250_deinit(MPU9250_Handle *sensor) {
    if(sensor) free(sensor);
}

MPU9250_Status_e MPU9250_readSensors(MPU9250_Handle *sensor) {
    uint8_t rawData[21];
    int16_t ax_raw, ay_raw, az_raw, t_raw, gx_raw, gy_raw, gz_raw;
    int16_t mx_raw, my_raw, mz_raw;

    if (!sensor) return MPU9250_ERROR_INVALID_HANDLE;

    if (!_readRegs(sensor, MPU6500_ACCEL_XOUT_H_ADDR, rawData, 14)) return MPU9250_ERROR_I2C;

    ax_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    ay_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    az_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    t_raw  = (int16_t)(((int16_t)rawData[6] << 8) | rawData[7]);
    gx_raw = (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
    gy_raw = (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
    gz_raw = (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);

    sensor->accelerometer.x = (float)ax_raw * sensor->accelScale - sensor->accelBias.x;
    sensor->accelerometer.y = (float)ay_raw * sensor->accelScale - sensor->accelBias.y;
    sensor->accelerometer.z = (float)az_raw * sensor->accelScale - sensor->accelBias.z;
    sensor->gyroscope.x = (float)gx_raw * sensor->gyroScale - sensor->gyroBias.x;
    sensor->gyroscope.y = (float)gy_raw * sensor->gyroScale - sensor->gyroBias.y;
    sensor->gyroscope.z = (float)gz_raw * sensor->gyroScale - sensor->gyroBias.z;
    sensor->temperature = ((float)t_raw / 333.87f) + 21.0f;

    if (sensor->hasMag) {
        if (!_magReadRegs(sensor, AK8963_ST1_ADDR, &rawData[14], 7)) return MPU9250_ERROR_I2C;
        if (rawData[14] & AK8963_ST1_DRDY_BIT) {
            mx_raw = (int16_t)(((int16_t)rawData[16] << 8) | rawData[15]);
            my_raw = (int16_t)(((int16_t)rawData[18] << 8) | rawData[17]);
            mz_raw = (int16_t)(((int16_t)rawData[20] << 8) | rawData[19]);

            sensor->magnetometer.x = (float)mx_raw * sensor->magScale * sensor->magAdjust.x - sensor->magBias.x;
            sensor->magnetometer.y = (float)my_raw * sensor->magScale * sensor->magAdjust.y - sensor->magBias.y;
            sensor->magnetometer.z = (float)mz_raw * sensor->magScale * sensor->magAdjust.z - sensor->magBias.z;
        }
    }
    return MPU9250_OK;
}

MPU9250_Status_e MPU9250_calibrate(MPU9250_Handle *sensor, const uint16_t numSamples) {
    MPU9250_Vector3D mean_acc = {0}, M2_acc = {0};
    MPU9250_Vector3D mean_gyr = {0}, M2_gyr = {0};
    MPU9250_Vector3D mean_mag = {0}, M2_mag = {0};
    MPU9250_Vector3D s_acc, d_acc, s_gyr, d_gyr, s_mag, d_mag;
    uint8_t rawData[21];
    uint16_t i;
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    int16_t mx_raw, my_raw, mz_raw;
    uint8_t accel_cfg_val;
    float fsr_g, lsb_per_g;
    float accel_scale_sq, gyro_scale_sq;
    float mag_scale_adj_x_sq, mag_scale_adj_y_sq, mag_scale_adj_z_sq;

    if (!sensor) return MPU9250_ERROR_INVALID_HANDLE;
    if (numSamples == 0) return MPU9250_ERROR_INVALID_PARAM;

    for (i = 1; i <= numSamples; ++i) {
        if(!_readRegs(sensor, MPU6500_ACCEL_XOUT_H_ADDR, rawData, 14)) return MPU9250_ERROR_I2C;
        ax_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        ay_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        az_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        gx_raw = (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
        gy_raw = (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
        gz_raw = (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);

        s_acc = (MPU9250_Vector3D){(float)ax_raw, (float)ay_raw, (float)az_raw};
        d_acc = (MPU9250_Vector3D){s_acc.x - mean_acc.x, s_acc.y - mean_acc.y, s_acc.z - mean_acc.z};
        mean_acc.x += d_acc.x / i; M2_acc.x += d_acc.x * (s_acc.x - mean_acc.x);
        mean_acc.y += d_acc.y / i; M2_acc.y += d_acc.y * (s_acc.y - mean_acc.y);
        mean_acc.z += d_acc.z / i; M2_acc.z += d_acc.z * (s_acc.z - mean_acc.z);

        s_gyr = (MPU9250_Vector3D){(float)gx_raw, (float)gy_raw, (float)gz_raw};
        d_gyr = (MPU9250_Vector3D){s_gyr.x - mean_gyr.x, s_gyr.y - mean_gyr.y, s_gyr.z - mean_gyr.z};
        mean_gyr.x += d_gyr.x / i; M2_gyr.x += d_gyr.x * (s_gyr.x - mean_gyr.x);
        mean_gyr.y += d_gyr.y / i; M2_gyr.y += d_gyr.y * (s_gyr.y - mean_gyr.y);
        mean_gyr.z += d_gyr.z / i; M2_gyr.z += d_gyr.z * (s_gyr.z - mean_gyr.z);

        if (sensor->hasMag) {
            if (_magReadRegs(sensor, AK8963_ST1_ADDR, &rawData[14], 7)) {
                if (rawData[14] & AK8963_ST1_DRDY_BIT) {
                    mx_raw = (int16_t)(((int16_t)rawData[16] << 8) | rawData[15]);
                    my_raw = (int16_t)(((int16_t)rawData[18] << 8) | rawData[17]);
                    mz_raw = (int16_t)(((int16_t)rawData[20] << 8) | rawData[19]);
                    s_mag = (MPU9250_Vector3D){(float)mx_raw, (float)my_raw, (float)mz_raw};
                    d_mag = (MPU9250_Vector3D){s_mag.x - mean_mag.x, s_mag.y - mean_mag.y, s_mag.z - mean_mag.z};
                    mean_mag.x += d_mag.x / i; M2_mag.x += d_mag.x * (s_mag.x - mean_mag.x);
                    mean_mag.y += d_mag.y / i; M2_mag.y += d_mag.y * (s_mag.y - mean_mag.y);
                    mean_mag.z += d_mag.z / i; M2_mag.z += d_mag.z * (s_mag.z - mean_mag.z);
                }
            }
        }
        Task_sleep(5);
    }

    if (!_readRegs(sensor, MPU6500_ACCEL_CONFIG_ADDR, &accel_cfg_val, 1)) return MPU9250_ERROR_I2C;
    fsr_g = 2.0f * (1 << ((accel_cfg_val >> 3) & 0x03));
    lsb_per_g = 32768.0f / fsr_g;
    mean_acc.z -= lsb_per_g;

    sensor->accelBias.x = mean_acc.x * sensor->accelScale;
    sensor->accelBias.y = mean_acc.y * sensor->accelScale;
    sensor->accelBias.z = mean_acc.z * sensor->accelScale;
    sensor->gyroBias.x = mean_gyr.x * sensor->gyroScale;
    sensor->gyroBias.y = mean_gyr.y * sensor->gyroScale;
    sensor->gyroBias.z = mean_gyr.z * sensor->gyroScale;

    accel_scale_sq = sensor->accelScale * sensor->accelScale;
    gyro_scale_sq = sensor->gyroScale * sensor->gyroScale;
    sensor->accelVariance.x = (M2_acc.x / (numSamples - 1)) * accel_scale_sq;
    sensor->accelVariance.y = (M2_acc.y / (numSamples - 1)) * accel_scale_sq;
    sensor->accelVariance.z = (M2_acc.z / (numSamples - 1)) * accel_scale_sq;
    sensor->gyroVariance.x = (M2_gyr.x / (numSamples - 1)) * gyro_scale_sq;
    sensor->gyroVariance.y = (M2_gyr.y / (numSamples - 1)) * gyro_scale_sq;
    sensor->gyroVariance.z = (M2_gyr.z / (numSamples - 1)) * gyro_scale_sq;

    if (sensor->hasMag) {
        sensor->magBias.x = mean_mag.x * sensor->magScale * sensor->magAdjust.x;
        sensor->magBias.y = mean_mag.y * sensor->magScale * sensor->magAdjust.y;
        sensor->magBias.z = mean_mag.z * sensor->magScale * sensor->magAdjust.z;
        mag_scale_adj_x_sq = powf(sensor->magScale * sensor->magAdjust.x, 2);
        mag_scale_adj_y_sq = powf(sensor->magScale * sensor->magAdjust.y, 2);
        mag_scale_adj_z_sq = powf(sensor->magScale * sensor->magAdjust.z, 2);
        sensor->magVariance.x = (M2_mag.x / (numSamples - 1)) * mag_scale_adj_x_sq;
        sensor->magVariance.y = (M2_mag.y / (numSamples - 1)) * mag_scale_adj_y_sq;
        sensor->magVariance.z = (M2_mag.z / (numSamples - 1)) * mag_scale_adj_z_sq;
    } else {
        memset(&sensor->magBias, 0, sizeof(MPU9250_Vector3D));
        memset(&sensor->magVariance, 0, sizeof(MPU9250_Vector3D));
    }
    return MPU9250_OK;
}

MPU9250_Status_e MPU9250_determineOrientationAxes(const MPU9250_Handle *sensor, MPU9250_Matrix3x3 *matrix, const ReferenceFrame_e frame) {
    uint8_t rawData[14];
    MPU9250_Vector3D vertical_vec = {0}, forward_vec = {0};
    MPU9250_Vector3D world_x = {0}, world_y = {0}, world_z = {0};
    MPU9250_Vector3D avg_acc = {0};
    int num_samples = 200;
    int i;
    float abs_x, abs_y, abs_z;
    MPU9250_Vector3D max_gyro = {0};
    uint32_t startTime;
    Types_FreqHz freq;
    int16_t gx, gy, gz;
    MPU9250_Matrix3x3 R_transpose;

    if (!sensor || !matrix) return MPU9250_ERROR_INVALID_PARAM;

    System_printf("\n--- Orientation Calibration Step 1: Finding Vertical Axis (%s) ---\n", (frame == FRAME_NED) ? "DOWN" : "UP");
    System_printf("Place the device on a flat, level surface and keep it still.\n");
    System_printf("Calibration will begin in 5 seconds...\n");
    System_flush();
    Task_sleep(5000);

    for (i = 0; i < num_samples; i++) {
        if(!_readRegs(sensor, MPU6500_ACCEL_XOUT_H_ADDR, rawData, 6)) return MPU9250_ERROR_I2C;
        avg_acc.x += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        avg_acc.y += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        avg_acc.z += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
        Task_sleep(10);
    }
    avg_acc.x /= num_samples; avg_acc.y /= num_samples; avg_acc.z /= num_samples;
    abs_x = fabsf(avg_acc.x); abs_y = fabsf(avg_acc.y); abs_z = fabsf(avg_acc.z);

    // Find the axis most aligned with gravity. The accelerometer measures the reaction
    // force, which is opposite to gravity. So if accel is positive on an axis,
    // that axis is pointing UP. The DOWN vector is therefore in the opposite direction.
    if (abs_x > abs_y && abs_x > abs_z) { vertical_vec.x = (avg_acc.x > 0) ? -1.0f : 1.0f; }
    else if (abs_y > abs_x && abs_y > abs_z) { vertical_vec.y = (avg_acc.y > 0) ? -1.0f : 1.0f; }
    else { vertical_vec.z = (avg_acc.z > 0) ? -1.0f : 1.0f; }

    System_printf("\n--- Orientation Calibration Step 2: Finding Forward Axis (NORTH) ---\n");
    System_printf("Pick up device, hold level, and do a sharp 'ROLL RIGHT' motion.\n");
    System_printf("Calibration will begin in 5 seconds...\n");
    System_flush();
    Task_sleep(5000);
    System_printf("PERFORM 'ROLL RIGHT' NOW!\n");
    System_flush();

    startTime = Timestamp_get32();
    Timestamp_getFreq(&freq);

    while (((Timestamp_get32() - startTime) * 1.0f / freq.lo) < 2.0f) {
        if(!_readRegs(sensor, MPU6500_GYRO_XOUT_H_ADDR, &rawData[8], 6)) return MPU9250_ERROR_I2C;
        gx = (int16_t)(((int16_t)rawData[8] << 8) | rawData[9]);
        gy = (int16_t)(((int16_t)rawData[10] << 8) | rawData[11]);
        gz = (int16_t)(((int16_t)rawData[12] << 8) | rawData[13]);
        if (gx > max_gyro.x) max_gyro.x = gx;
        if (gy > max_gyro.y) max_gyro.y = gy;
        if (gz > max_gyro.z) max_gyro.z = gz;
        Task_sleep(10);
    }

    // A 'ROLL RIGHT' motion produces a positive angular velocity around the forward axis.
    abs_x = fabsf(max_gyro.x); abs_y = fabsf(max_gyro.y); abs_z = fabsf(max_gyro.z);
    if (abs_x > abs_y && abs_x > abs_z) { forward_vec.x = (max_gyro.x > 0) ? 1.0f : -1.0f; }
    else if (abs_y > abs_x && abs_y > abs_z) { forward_vec.y = (max_gyro.y > 0) ? 1.0f : -1.0f; }
    else { forward_vec.z = (max_gyro.z > 0) ? 1.0f : -1.0f; }

    // Build the rotation matrix rows based on the selected reference frame
    // using the right-hand rule. The matrix rows are the world-frame basis
    // vectors expressed in the sensor's body-frame coordinates.
    if (frame == FRAME_NED) {
        // NED: X=North, Y=East, Z=Down
        // Right-hand rule: North(X) x East(Y) = Down(Z) => East(Y) = Down(Z) x North(X)
        world_x = forward_vec;      // North
        world_z = vertical_vec;     // Down
        world_y = _vec_cross(&world_z, &world_x); // East
    } else { // FRAME_ENU
        // ENU: X=East, Y=North, Z=Up
        // Right-hand rule: East(X) x North(Y) = Up(Z) => East(X) = North(Y) x Up(Z)
        world_y = forward_vec;      // North
        // The vertical_vec is DOWN, so UP is its opposite.
        world_z.x = -vertical_vec.x;
        world_z.y = -vertical_vec.y;
        world_z.z = -vertical_vec.z;
        world_x = _vec_cross(&world_y, &world_z); // East
    }

    // Assign the world basis vectors to the rows of the rotation matrix
    matrix->m[0][0] = world_x.x; matrix->m[0][1] = world_x.y; matrix->m[0][2] = world_x.z;
    matrix->m[1][0] = world_y.x; matrix->m[1][1] = world_y.y; matrix->m[1][2] = world_y.z;
    matrix->m[2][0] = world_z.x; matrix->m[2][1] = world_z.y; matrix->m[2][2] = world_z.z;

    return MPU9250_OK;
}

void MPU9250_applyRotation(const MPU9250_Matrix3x3 *matrix, MPU9250_Vector3D *vector) {
    MPU9250_Vector3D temp;
    if (!matrix || !vector) return;
    temp = *vector;
    vector->x = matrix->m[0][0] * temp.x + matrix->m[0][1] * temp.y + matrix->m[0][2] * temp.z;
    vector->y = matrix->m[1][0] * temp.x + matrix->m[1][1] * temp.y + matrix->m[1][2] * temp.z;
    vector->z = matrix->m[2][0] * temp.x + matrix->m[2][1] * temp.y + matrix->m[2][2] * temp.z;
}

MPU9250_Status_e MPU9250_SensorFusionInit(MPU9250_Handle *sensor, FusionAlgorithm_e algorithm, const void *params) {
    if (!sensor) return MPU9250_ERROR_INVALID_HANDLE;
    if (!params) return MPU9250_ERROR_INVALID_PARAM;

    sensor->fusion.algorithm = algorithm;
    switch(algorithm) {
        case FUSION_ALGORITHM_COMPLEMENTARY:
            sensor->fusion.params.comp = *(ComplementaryFilterParams*)params; break;
        case FUSION_ALGORITHM_MADGWICK:
            sensor->fusion.params.madgwick = *(MadgwickFilterParams*)params; break;
        case FUSION_ALGORITHM_MAHONY:
            sensor->fusion.params.mahony = *(MahonyFilterParams*)params; break;
    }

    sensor->fusion.orientation = (MPU9250_Quaternion){ .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
    sensor->fusion.euler = (MPU9250_EulerAngles){ .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f };
    sensor->fusion.mahonyIntegralError = (MPU9250_Vector3D){0};
    return MPU9250_OK;
}

MPU9250_Status_e MPU9250_SensorFusionUpdate(MPU9250_Handle *sensor, FusionDOF_e dof, float dt) {
    if (!sensor) return MPU9250_ERROR_INVALID_HANDLE;
    if (dt <= 0) return MPU9250_ERROR_INVALID_PARAM;

    switch (sensor->fusion.algorithm) {
        case FUSION_ALGORITHM_COMPLEMENTARY:
            _updateComplementaryFilter(sensor, dof, dt); break;
        case FUSION_ALGORITHM_MADGWICK:
            _updateMadgwickFilter(sensor, dof, dt); break;
        case FUSION_ALGORITHM_MAHONY:
            _updateMahonyFilter(sensor, dof, dt); break;
    }
    return MPU9250_OK;
}

MPU9250_Status_e MPU9250_getOrientation(MPU9250_Handle *sensor, MPU9250_Quaternion *q) {
    if (!sensor || !q) return MPU9250_ERROR_INVALID_PARAM;
    *q = sensor->fusion.orientation;
    return MPU9250_OK;
}

MPU9250_Status_e MPU9250_getEulerAngles(MPU9250_Handle *sensor, MPU9250_EulerAngles *angles) {
    MPU9250_Quaternion q;
    if (!sensor || !angles) return MPU9250_ERROR_INVALID_PARAM;
    q = sensor->fusion.orientation;

    if (sensor->fusion.algorithm == FUSION_ALGORITHM_COMPLEMENTARY) {
        *angles = sensor->fusion.euler;
    } else {
        angles->roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
        angles->pitch = asinf(2.0f * (q.w * q.y - q.z * q.x));
        angles->yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    }
    return MPU9250_OK;
}
