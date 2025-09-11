// ================================================================================
// 
// File Name           : MPU9250.h
// Target Devices      : MSP-EXP432E401Y
// Description         : 
//     Header file for MPU9250 IMU sensor driver
//      defines constants, register maps, function prototypes, and hardware drivers
//      for interacting via I2C and a GPIO on TI-RTOS
// 
// Author              : Adrià Babiano Novella
// Create Date         : 2024-12-01
// Revision            : v2.0
// ================================================================================

/**
 * @file MPU9250.h
 * @brief Header file for the MPU9250 IMU sensor driver.
 *
 * Defines constants, register maps, function prototypes, and includes necessary
 * for interacting with the MPU9250 sensor via I2C on a TI-RTOS platform.
 * 
 * Same library can be used for MPU6500 IMU sensor, the core is identical
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>

#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Timestamp.h>

// --- Constants ---
// --- Device Specific ---
#define MPU9250_I2C_ADDR_1          0x68
#define MPU9250_I2C_ADDR_2          0x69
#define MPU6500_WHO_AM_I            0x70
#define MPU9250_WHO_AM_I            0x71
#define MPU9255_WHO_AM_I            0x73

// --- MPU6500 REGISTER MAP ---
#define MPU6500_SELF_TEST_X_GYRO_ADDR   0x00
#define MPU6500_SELF_TEST_Y_GYRO_ADDR   0x01
#define MPU6500_SELF_TEST_Z_GYRO_ADDR   0x02
#define MPU6500_SELF_TEST_X_ACCEL_ADDR  0x0D
#define MPU6500_SELF_TEST_Y_ACCEL_ADDR  0x0E
#define MPU6500_SELF_TEST_Z_ACCEL_ADDR  0x0F
#define MPU6500_XG_OFFSET_H_ADDR        0x13
#define MPU6500_XG_OFFSET_L_ADDR        0x14
#define MPU6500_YG_OFFSET_H_ADDR        0x15
#define MPU6500_YG_OFFSET_L_ADDR        0x16
#define MPU6500_ZG_OFFSET_H_ADDR        0x17
#define MPU6500_ZG_OFFSET_L_ADDR        0x18
#define MPU6500_SMPLRT_DIV_ADDR         0x19
#define MPU6500_CONFIG_ADDR             0x1A
#define MPU6500_GYRO_CONFIG_ADDR        0x1B
#define MPU6500_ACCEL_CONFIG_ADDR       0x1C
#define MPU6500_ACCEL_CONFIG_2_ADDR     0x1D
#define MPU6500_LP_ACCEL_ODR_ADDR       0x1E
#define MPU6500_WOM_THR_ADDR            0x1F
#define MPU6500_FIFO_EN_ADDR            0x23
#define MPU6500_I2C_MST_CTRL_ADDR       0x24
#define MPU6500_I2C_SLV0_ADDR_ADDR      0x25
#define MPU6500_I2C_SLV0_REG_ADDR       0x26
#define MPU6500_I2C_SLV0_CTRL_ADDR      0x27
#define MPU6500_I2C_SLV1_ADDR_ADDR      0x28
#define MPU6500_I2C_SLV1_REG_ADDR       0x29
#define MPU6500_I2C_SLV1_CTRL_ADDR      0x2A
#define MPU6500_I2C_SLV2_ADDR_ADDR      0x2B
#define MPU6500_I2C_SLV2_REG_ADDR       0x2C
#define MPU6500_I2C_SLV2_CTRL_ADDR      0x2D
#define MPU6500_I2C_SLV3_ADDR_ADDR      0x2E
#define MPU6500_I2C_SLV3_REG_ADDR       0x2F
#define MPU6500_I2C_SLV3_CTRL_ADDR      0x30
#define MPU6500_I2C_SLV4_ADDR_ADDR      0x31
#define MPU6500_I2C_SLV4_REG_ADDR       0x32
#define MPU6500_I2C_SLV4_DO_ADDR        0x33
#define MPU6500_I2C_SLV4_CTRL_ADDR      0x34
#define MPU6500_I2C_SLV4_DI_ADDR        0x35
#define MPU6500_I2C_MST_STATUS_ADDR     0x36
#define MPU6500_INT_PIN_CFG_ADDR        0x37
#define MPU6500_INT_ENABLE_ADDR         0x38
#define MPU6500_INT_STATUS_ADDR         0x3A
#define MPU6500_ACCEL_XOUT_H_ADDR       0x3B
#define MPU6500_ACCEL_XOUT_L_ADDR       0x3C
#define MPU6500_ACCEL_YOUT_H_ADDR       0x3D
#define MPU6500_ACCEL_YOUT_L_ADDR       0x3E
#define MPU6500_ACCEL_ZOUT_H_ADDR       0x3F
#define MPU6500_ACCEL_ZOUT_L_ADDR       0x40
#define MPU6500_TEMP_OUT_H_ADDR         0x41
#define MPU6500_TEMP_OUT_L_ADDR         0x42
#define MPU6500_GYRO_XOUT_H_ADDR        0x43
#define MPU6500_GYRO_XOUT_L_ADDR        0x44
#define MPU6500_GYRO_YOUT_H_ADDR        0x45
#define MPU6500_GYRO_YOUT_L_ADDR        0x46
#define MPU6500_GYRO_ZOUT_H_ADDR        0x47
#define MPU6500_GYRO_ZOUT_L_ADDR        0x48
#define MPU6500_EXT_SENS_DATA_00_ADDR   0x49
#define MPU6500_EXT_SENS_DATA_01_ADDR   0x4A
#define MPU6500_EXT_SENS_DATA_02_ADDR   0x4B
#define MPU6500_EXT_SENS_DATA_03_ADDR   0x4C
#define MPU6500_EXT_SENS_DATA_04_ADDR   0x4D
#define MPU6500_EXT_SENS_DATA_05_ADDR   0x4E
#define MPU6500_EXT_SENS_DATA_06_ADDR   0x4F
#define MPU6500_EXT_SENS_DATA_07_ADDR   0x50
#define MPU6500_EXT_SENS_DATA_08_ADDR   0x51
#define MPU6500_EXT_SENS_DATA_09_ADDR   0x52
#define MPU6500_EXT_SENS_DATA_10_ADDR   0x53
#define MPU6500_EXT_SENS_DATA_11_ADDR   0x54
#define MPU6500_EXT_SENS_DATA_12_ADDR   0x55
#define MPU6500_EXT_SENS_DATA_13_ADDR   0x56
#define MPU6500_EXT_SENS_DATA_14_ADDR   0x57
#define MPU6500_EXT_SENS_DATA_15_ADDR   0x58
#define MPU6500_EXT_SENS_DATA_16_ADDR   0x59
#define MPU6500_EXT_SENS_DATA_17_ADDR   0x5A
#define MPU6500_EXT_SENS_DATA_18_ADDR   0x5B
#define MPU6500_EXT_SENS_DATA_19_ADDR   0x5C
#define MPU6500_EXT_SENS_DATA_20_ADDR   0x5D
#define MPU6500_EXT_SENS_DATA_21_ADDR   0x5E
#define MPU6500_EXT_SENS_DATA_22_ADDR   0x5F
#define MPU6500_EXT_SENS_DATA_23_ADDR   0x60
#define MPU6500_I2C_SLV0_DO_ADDR        0x63
#define MPU6500_I2C_SLV1_DO_ADDR        0x64
#define MPU6500_I2C_SLV2_DO_ADDR        0x65
#define MPU6500_I2C_SLV3_DO_ADDR        0x66
#define MPU6500_I2C_MST_DELAY_CTRL_ADDR 0x67
#define MPU6500_SIGNAL_PATH_RESET_ADDR  0x68
#define MPU6500_MOT_DETECT_CTRL_ADDR    0x69
#define MPU6500_USER_CTRL_ADDR          0x6A
#define MPU6500_PWR_MGMT_1_ADDR         0x6B
#define MPU6500_PWR_MGMT_2_ADDR         0x6C
#define MPU6500_FIFO_COUNTH_ADDR        0x72
#define MPU6500_FIFO_COUNTL_ADDR        0x73
#define MPU6500_FIFO_R_W_ADDR           0x74
#define MPU6500_WHO_AM_I_ADDR           0x75
#define MPU6500_XA_OFFSET_H_ADDR        0x77
#define MPU6500_XA_OFFSET_L_ADDR        0x78
#define MPU6500_YA_OFFSET_H_ADDR        0x7A
#define MPU6500_YA_OFFSET_L_ADDR        0x7B
#define MPU6500_ZA_OFFSET_H_ADDR        0x7D
#define MPU6500_ZA_OFFSET_L_ADDR        0x7E

// --- REGISTER BIT DEFINITIONS ---

// CONFIG (0x1A)
#define MPU6500_CONFIG_FIFO_MODE_BIT    (1 << 6) // When set, FIFO will continue writing when full, overwriting oldest data
#define MPU6500_CONFIG_EXT_SYNC_SET_MASK 0b00111000 // External Frame Synchronization (FSYNC)
#define MPU6500_CONFIG_DLPF_CFG_MASK    0b00000111 // Digital Low Pass Filter configuration

// GYRO_CONFIG (0x1B)
#define MPU6500_GYRO_CONFIG_XG_ST_BIT       (1 << 7) // X Gyro self-test
#define MPU6500_GYRO_CONFIG_YG_ST_BIT       (1 << 6) // Y Gyro self-test
#define MPU6500_GYRO_CONFIG_ZG_ST_BIT       (1 << 5) // Z Gyro self-test
#define MPU6500_GYRO_CONFIG_FS_SEL_MASK     0b00011000 // Gyro full-scale select
#define MPU6500_GYRO_FS_SEL_250DPS          (0 << 3)
#define MPU6500_GYRO_FS_SEL_500DPS          (1 << 3)
#define MPU6500_GYRO_FS_SEL_1000DPS         (2 << 3)
#define MPU6500_GYRO_FS_SEL_2000DPS         (3 << 3)
#define MPU6500_GYRO_CONFIG_FCHOICE_B_MASK  0b00000011 // Used to bypass DLPF

// ACCEL_CONFIG (0x1C)
#define MPU6500_ACCEL_CONFIG_AX_ST_EN_BIT   (1 << 7) // X Accel self-test enable
#define MPU6500_ACCEL_CONFIG_AY_ST_EN_BIT   (1 << 6) // Y Accel self-test enable
#define MPU6500_ACCEL_CONFIG_AZ_ST_EN_BIT   (1 << 5) // Z Accel self-test enable
#define MPU6500_ACCEL_CONFIG_FS_SEL_MASK    0b00011000 // Accel full-scale select
#define MPU6500_ACCEL_FS_SEL_2G             (0 << 3)
#define MPU6500_ACCEL_FS_SEL_4G             (1 << 3)
#define MPU6500_ACCEL_FS_SEL_8G             (2 << 3)
#define MPU6500_ACCEL_FS_SEL_16G            (3 << 3)

// ACCEL_CONFIG_2 (0x1D)
#define MPU6500_ACCEL_CONFIG2_FCHOICE_B_BIT (1 << 3) // Used to bypass DLPF
#define MPU6500_ACCEL_CONFIG2_A_DLPFCFG_MASK 0b00000111 // Accel Digital Low Pass Filter configuration

// FIFO_EN (0x23)
#define MPU6500_FIFO_EN_TEMP_BIT        (1 << 7) // Temperature sensor data to FIFO
#define MPU6500_FIFO_EN_XG_BIT          (1 << 6) // X Gyro data to FIFO
#define MPU6500_FIFO_EN_YG_BIT          (1 << 5) // Y Gyro data to FIFO
#define MPU6500_FIFO_EN_ZG_BIT          (1 << 4) // Z Gyro data to FIFO
#define MPU6500_FIFO_EN_ACCEL_BIT       (1 << 3) // Accelerometer data to FIFO
#define MPU6500_FIFO_EN_SLV2_BIT        (1 << 2) // Slave 2 data to FIFO
#define MPU6500_FIFO_EN_SLV1_BIT        (1 << 1) // Slave 1 data to FIFO
#define MPU6500_FIFO_EN_SLV0_BIT        (1 << 0) // Slave 0 data to FIFO

// I2C_MST_CTRL (0x24)
#define MPU6500_I2C_MST_CTRL_MULT_MST_EN_BIT (1 << 7)
#define MPU6500_I2C_MST_CTRL_P_NSR_BIT       (1 << 4) // Stop/Restart control for read/write
#define MPU6500_I2C_MST_CTRL_SLV_3_FIFO_EN_BIT (1 << 5)
#define MPU6500_I2C_MST_CTRL_CLK_MASK        0b00001111 // I2C master clock speed

// I2C_SLV0_ADDR (0x25) & others
#define MPU6500_I2C_SLV_RNW_BIT         (1 << 7) // Read/Write bit: 1 for read, 0 for write
#define MPU6500_I2C_SLV_ID_MASK         0b01111111 // I2C slave address

// I2C_SLV0_CTRL (0x27) & others
#define MPU6500_I2C_SLV_EN_BIT          (1 << 7) // Enable reading from this slave
#define MPU6500_I2C_SLV_BYTE_SW_BIT     (1 << 6) // Byte-swap high and low bytes of word
#define MPU6500_I2C_SLV_REG_DIS_BIT     (1 << 5) // 1 = Do not write slave register address
#define MPU6500_I2C_SLV_GRP_BIT         (1 << 4) // Group read/writes
#define MPU6500_I2C_SLV_LENG_MASK       0b00001111 // Number of bytes to read

// INT_PIN_CFG (0x37)
#define MPU6500_INT_PIN_CFG_ACTL_BIT        (1 << 7) // INT pin logic level (0=active high, 1=active low)
#define MPU6500_INT_PIN_CFG_OPEN_BIT        (1 << 6) // INT pin open-drain
#define MPU6500_INT_PIN_CFG_LATCH_EN_BIT    (1 << 5) // Latch INT pin until cleared
#define MPU6500_INT_PIN_CFG_ANYRD_2CLEAR_BIT (1 << 4) // Clear interrupt on any read operation
#define MPU6500_INT_PIN_CFG_FSYNC_LVL_BIT   (1 << 3) // FSYNC pin logic level
#define MPU6500_INT_PIN_CFG_FSYNC_EN_BIT    (1 << 2) // FSYNC pin interrupt enable
#define MPU6500_INT_PIN_CFG_BYPASS_EN_BIT   (1 << 1) // Enable I2C bypass to access magnetometer

// INT_ENABLE (0x38)
#define MPU6500_INT_ENABLE_WOM_BIT          (1 << 6) // Wake on Motion interrupt enable
#define MPU6500_INT_ENABLE_FIFO_OFLOW_BIT   (1 << 4) // FIFO overflow interrupt enable
#define MPU6500_INT_ENABLE_FSYNC_BIT        (1 << 3) // FSYNC interrupt enable
#define MPU6500_INT_ENABLE_RAW_RDY_BIT      (1 << 0) // Raw data ready interrupt enable

// INT_STATUS (0x3A) - Read-only
#define MPU6500_INT_STATUS_WOM_BIT          (1 << 6) // Wake on Motion interrupt status
#define MPU6500_INT_STATUS_FIFO_OFLOW_BIT   (1 << 4) // FIFO overflow interrupt status
#define MPU6500_INT_STATUS_FSYNC_BIT        (1 << 3) // FSYNC interrupt status
#define MPU6500_INT_STATUS_RAW_RDY_BIT      (1 << 0) // Raw data ready interrupt status

// USER_CTRL (0x6A)
#define MPU6500_USER_CTRL_FIFO_EN_BIT       (1 << 6) // Enable FIFO buffer
#define MPU6500_USER_CTRL_I2C_MST_EN_BIT    (1 << 5) // Enable I2C Master mode
#define MPU6500_USER_CTRL_I2C_IF_DIS_BIT    (1 << 4) // Disable I2C slave interface
#define MPU6500_USER_CTRL_FIFO_RESET_BIT    (1 << 2) // Reset FIFO
#define MPU6500_USER_CTRL_I2C_MST_RESET_BIT (1 << 1) // Reset I2C Master
#define MPU6500_USER_CTRL_SIG_COND_RESET_BIT (1 << 0) // Reset all signal paths

// PWR_MGMT_1 (0x6B)
#define MPU6500_PWR_MGMT_1_H_RESET_BIT      (1 << 7) // Reset device
#define MPU6500_PWR_MGMT_1_SLEEP_BIT        (1 << 6) // Enter sleep mode
#define MPU6500_PWR_MGMT_1_CYCLE_BIT        (1 << 5) // Cycle between sleep and active
#define MPU6500_PWR_MGMT_1_GYRO_STANDBY_BIT (1 << 4) // Put gyro in standby mode
#define MPU6500_PWR_MGMT_1_PD_PTAT_BIT      (1 << 3) // Power down temperature sensor
#define MPU6500_PWR_MGMT_1_CLKSEL_MASK      0b00000111 // Clock source select
#define MPU6500_CLKSEL_INTERNAL_20MHZ       0x00
#define MPU6500_CLKSEL_AUTO_SELECT          0x01 // Auto selects best available clock source

// PWR_MGMT_2 (0x6C)
#define MPU6500_PWR_MGMT_2_DISABLE_XG_BIT   (1 << 5) // Disable X Gyro
#define MPU6500_PWR_MGMT_2_DISABLE_YG_BIT   (1 << 4) // Disable Y Gyro
#define MPU6500_PWR_MGMT_2_DISABLE_ZG_BIT   (1 << 3) // Disable Z Gyro
#define MPU6500_PWR_MGMT_2_DISABLE_XA_BIT   (1 << 2) // Disable X Accel
#define MPU6500_PWR_MGMT_2_DISABLE_YA_BIT   (1 << 1) // Disable Y Accel
#define MPU6500_PWR_MGMT_2_DISABLE_ZA_BIT   (1 << 0) // Disable Z Accel
#define MPU6500_PWR_MGMT_2_DISABLE_ALL_GYRO_MASK 0b00111000
#define MPU6500_PWR_MGMT_2_DISABLE_ALL_ACCEL_MASK 0b00000111

// --- AK8963 Address
#define AK8963_I2C_ADDR                 0x0C

// --- AK8963 REGISTER MAP ---
#define AK8963_WIA_ADDR                 0x00
#define AK8963_INFO_ADDR                0x01
#define AK8963_ST1_ADDR                 0x02
#define AK8963_HXL_ADDR                 0x03
#define AK8963_HXH_ADDR                 0x04
#define AK8963_HYL_ADDR                 0x05
#define AK8963_HYH_ADDR                 0x06
#define AK8963_HZL_ADDR                 0x07
#define AK8963_HZH_ADDR                 0x08
#define AK8963_ST2_ADDR                 0x09
#define AK8963_CNTL1_ADDR               0x0A
#define AK8963_CNTL2_ADDR               0x0B
#define AK8963_ASTC_ADDR                0x0C
#define AK8963_I2CDIS_ADDR              0x0F
#define AK8963_ASAX_ADDR                0x10
#define AK8963_ASAY_ADDR                0x11
#define AK8963_ASAZ_ADDR                0x12

// --- REGISTER BIT DEFINITIONS ---

// WIA (0x00) - Who I Am
#define AK8963_WHO_AM_I_VALUE           0x48 // Expected value for AK8963

// ST1 (0x02) - Status 1
#define AK8963_ST1_DRDY_BIT             (1 << 0) // Data Ready
#define AK8963_ST1_DOR_BIT              (1 << 1) // Data Overrun

// ST2 (0x09) - Status 2
#define AK8963_ST2_HOFL_BIT             (1 << 3) // Magnetic sensor overflow
#define AK8963_ST2_BITM_BIT             (1 << 4) // Output bit size (mirror of CNTL1 BIT)

// CNTL1 (0x0A) - Control 1
#define AK8963_CNTL1_MODE_MASK          0b00001111 // Operation mode setting
#define AK8963_MODE_POWER_DOWN          0b0000 // 0x00
#define AK8963_MODE_SINGLE_MEASUREMENT  0b0001 // 0x01
#define AK8963_MODE_CONTINUOUS_8HZ      0b0010 // 0x02
#define AK8963_MODE_CONTINUOUS_100HZ    0b0110 // 0x06
#define AK8963_MODE_EXTERNAL_TRIGGER    0b0100 // 0x04
#define AK8963_MODE_SELF_TEST           0b1000 // 0x08
#define AK8963_MODE_FUSE_ROM            0b1111 // 0x0F
#define AK8963_CNTL1_BIT_MASK           (1 << 4) // Output bit setting
#define AK8963_BIT_14                   (0 << 4) // 14-bit output
#define AK8963_BIT_16                   (1 << 4) // 16-bit output

// CNTL2 (0x0B) - Control 2
#define AK8963_CNTL2_SRST_BIT           (1 << 0) // Soft Reset

// ASTC (0x0C) - Self-Test Control
#define AK8963_ASTC_SELF_BIT            (1 << 6) // Self test

// I2CDIS (0x0F) - I2C Disable
#define AK8963_I2CDIS_BIT               (1 << 0) // Disable I2C interface

// --- Physical Constants ---
#define G_MPS2          9.80665f
#define RAD_TO_DEG      57.2957795f
#define DEG_TO_RAD      1.0f/RAD_TO_DEG

// --- Custom Data Types ---

/** @brief Structure to represent a 3-dimensional vector. */
typedef struct MPU9250_Vector3D {
    float x; /**< X-component */
    float y; /**< Y-component */
    float z; /**< Z-component */
} MPU9250_Vector3D;

/** @brief Structure to represent a Quaternion for 3D orientation. */
typedef struct MPU9250_Quaternion {
    float w; /**< Scalar component */
    float x; /**< X-vector component */
    float y; /**< Y-vector component */
    float z; /**< Z-vector component */
} MPU9250_Quaternion;

/** @brief Structure to represent Euler angles. Angles are in radians. */
typedef struct MPU9250_EulerAngles {
    float roll;  /**< Rotation around the X-axis (phi), in radians */
    float pitch; /**< Rotation around the Y-axis (theta), in radians */
    float yaw;   /**< Rotation around the Z-axis (psi), in radians */
} MPU9250_EulerAngles;

/** @brief Structure to represent a 3x3 rotation matrix. */
typedef struct MPU9250_Matrix3x3 {
    float m[3][3]; /**< 3x3 matrix elements */
} MPU9250_Matrix3x3;

/** @brief Configuration parameters for the Complementary Filter. */
typedef struct ComplementaryFilterParams {
    float alpha; /**< Filter coefficient. A higher value trusts the gyroscope more (e.g., 0.98). */
} ComplementaryFilterParams;

/** @brief Configuration parameters for the Madgwick Filter. */
typedef struct MadgwickFilterParams {
    float beta; /**< Filter gain. Represents gyroscope measurement error (e.g., 0.1). */
} MadgwickFilterParams;

/** @brief Configuration parameters for the Mahony Filter. */
typedef struct MahonyFilterParams {
    float Kp; /**< Proportional gain for attitude correction (e.g., 2.5). */
    float Ki; /**< Integral gain to correct gyroscope bias (e.g., 0.05). */
} MahonyFilterParams;

/** @brief Union to hold the parameters for the selected fusion algorithm. */
typedef union FusionParams {
    ComplementaryFilterParams comp;
    MadgwickFilterParams      madgwick;
    MahonyFilterParams        mahony;
} FusionParams;

/** @brief Internal state for the sensor fusion module. */
typedef struct MPU9250_FusionState {
    FusionAlgorithm_e algorithm;  /**< The selected fusion algorithm. */
    FusionParams params;          /**< Parameters for the selected algorithm. */
    MPU9250_Quaternion orientation; /**< The calculated orientation as a quaternion. */
    MPU9250_EulerAngles euler;    /**< The calculated orientation as Euler angles (in radians). */
    MPU9250_Vector3D mahonyIntegralError; /**< Integral error term for the Mahony filter. */
} MPU9250_FusionState;

/** @brief Status codes returned by driver functions. */
typedef enum MPU9250_Status_e {
    MPU9250_OK,                 /**< Operation was successful. */
    MPU9250_ERROR_INVALID_HANDLE,/**< The provided sensor handle was NULL. */
    MPU9250_ERROR_MALLOC,       /**< Failed to allocate memory for the handle. */
    MPU9250_ERROR_I2C,          /**< An I2C communication error occurred. */
    MPU9250_ERROR_WHO_AM_I,     /**< The device WHO_AM_I value was incorrect. */
    MPU9250_ERROR_TIMEOUT,      /**< A timeout occurred waiting for a sensor event. */
    MPU9250_ERROR_INVALID_PARAM,/**< An invalid parameter was passed to a function. */
    MPU9250_ERROR_NOT_SUPPORTED,/**< The requested feature is not supported by the detected device. */
} MPU9250_Status_e;

/** @brief Accelerometer Full Scale Range (FSR) settings. */
typedef enum AccelFSR_e {
    ACCEL_FS_2G  = MPU6500_ACCEL_FS_SEL_2G,
    ACCEL_FS_4G  = MPU6500_ACCEL_FS_SEL_4G,
    ACCEL_FS_8G  = MPU6500_ACCEL_FS_SEL_8G,
    ACCEL_FS_16G = MPU6500_ACCEL_FS_SEL_16G
} AccelFSR_e;

/** @brief Gyroscope Full Scale Range (FSR) settings. */
typedef enum GyroFSR_e {
    GYRO_FS_250DPS  = MPU6500_GYRO_FS_SEL_250DPS,
    GYRO_FS_500DPS  = MPU6500_GYRO_FS_SEL_500DPS,
    GYRO_FS_1000DPS = MPU6500_GYRO_FS_SEL_1000DPS,
    GYRO_FS_2000DPS = MPU6500_GYRO_FS_SEL_2000DPS
} GyroFSR_e;

/** @brief Accelerometer Digital Low Pass Filter (DLPF) bandwidth settings. */
typedef enum AccelDLPF_e {
    ACCEL_DLPF_460HZ, // Corresponds to register value 0 or 7
    ACCEL_DLPF_184HZ, // Corresponds to register value 1
    ACCEL_DLPF_92HZ,  // Corresponds to register value 2
    ACCEL_DLPF_41HZ,  // Corresponds to register value 3
    ACCEL_DLPF_20HZ,  // Corresponds to register value 4
    ACCEL_DLPF_10HZ,  // Corresponds to register value 5
    ACCEL_DLPF_5HZ    // Corresponds to register value 6
} AccelDLPF_e;

/** @brief Gyroscope Digital Low Pass Filter (DLPF) bandwidth settings. */
typedef enum GyroDLPF_e {
    GYRO_DLPF_250HZ, // Corresponds to register value 0
    GYRO_DLPF_184HZ, // Corresponds to register value 1
    GYRO_DLPF_92HZ,  // Corresponds to register value 2
    GYRO_DLPF_41HZ,  // Corresponds to register value 3
    GYRO_DLPF_20HZ,  // Corresponds to register value 4
    GYRO_DLPF_10HZ,  // Corresponds to register value 5
    GYRO_DLPF_5HZ    // Corresponds to register value 6
} GyroDLPF_e;

/** @brief Magnetometer (AK8963) continuous measurement mode settings. */
typedef enum MagMode_e {
    MAG_MODE_POWER_DOWN = AK8963_MODE_POWER_DOWN,
    MAG_MODE_SINGLE     = AK8963_MODE_SINGLE_MEASUREMENT,
    MAG_MODE_8HZ        = AK8963_MODE_CONTINUOUS_8HZ,
    MAG_MODE_100HZ      = AK8963_MODE_CONTINUOUS_100HZ
} MagMode_e;

/** @brief Magnetometer (AK8963) output bit resolution settings. */
typedef enum  MagOutput_e {
    MAG_OUT_14_BIT = AK8963_BIT_14,
    MAG_OUT_16_BIT = AK8963_BIT_16
} MagOutput_e;

/** @brief Sensor fusion algorithm selection. */
typedef enum FusionAlgorithm_e {
    FUSION_ALGORITHM_COMPLEMENTARY, /**< Simple Complementary Filter. */
    FUSION_ALGORITHM_MADGWICK,      /**< Madgwick Gradient Descent AHRS. */
    FUSION_ALGORITHM_MAHONY         /**< Mahony PI Controller AHRS. */
} FusionAlgorithm_e;

/** @brief Degrees of Freedom (DOF) selection for sensor fusion. */
typedef enum FusionDOF_e {
    FUSION_DOF_6, /**< Use Accelerometer and Gyroscope only. */
    FUSION_DOF_9  /**< Use Accelerometer, Gyroscope, and Magnetometer. */
} FusionDOF_e;

/** @brief Coordinate system reference frames for orientation calibration. */
typedef enum ReferenceFrame_e {
    FRAME_NED, /**< North-East-Down: X-axis -> North, Y-axis -> East, Z-axis -> Down. */
    FRAME_ENU  /**< East-North-Up: X-axis -> East, Y-axis -> North, Z-axis -> Up. */
} ReferenceFrame_e;

typedef enum LogicLevel_e {
    ACTIVE_HIGH,
    ACTIVE_LOW
} LogicLevel_e;

typedef enum PinMode_e {
    PUSH_PULL,
    OPEN_DRAIN
} PinMode_e;

typedef struct InterruptPinConfig {
    LogicLevel_e level;          /**< Pin logic level when active (ACTIVE_LOW is typical for open-drain) */
    PinMode_e mode;              /**< PUSH_PULL or OPEN_DRAIN */
    bool latching;               /**< If true, interrupt pin stays active until status is read */
} InterruptPinConfig;

typedef enum InterruptSource_e {
    INT_ENABLE_WOM        = MPU6500_INT_ENABLE_WOM_BIT,        // Wake on Motion
    INT_ENABLE_FIFO_OFLOW = MPU6500_INT_ENABLE_FIFO_OFLOW_BIT, // FIFO Overflow
    INT_ENABLE_FSYNC      = MPU6500_INT_ENABLE_FSYNC_BIT,      // FSYNC
    INT_ENABLE_RAW_RDY    = MPU6500_INT_ENABLE_RAW_RDY_BIT     // Raw Data Ready
} InterruptSource_e;

typedef enum FIFO_Mode_e {
    FIFO_MODE_STOP_ON_FULL,
    FIFO_MODE_OVERWRITE
} FIFO_Mode_e;

typedef enum FIFO_Source_e {
    FIFO_EN_TEMP = MPU6500_FIFO_EN_TEMP_BIT,  // Temperature
    FIFO_EN_XG   = MPU6500_FIFO_EN_XG_BIT,    // Gyro X
    FIFO_EN_YG   = MPU6500_FIFO_EN_YG_BIT,    // Gyro Y
    FIFO_EN_ZG   = MPU6500_FIFO_EN_ZG_BIT,    // Gyro Z
    FIFO_EN_ACCEL= MPU6500_FIFO_EN_ACCEL_BIT, // Accelerometer (all 3 axes)
    FIFO_EN_SLV2 = MPU6500_FIFO_EN_SLV2_BIT,  // I2C Slave 2
    FIFO_EN_SLV1 = MPU6500_FIFO_EN_SLV1_BIT,  // I2C Slave 1
    FIFO_EN_SLV0 = MPU6500_FIFO_EN_SLV0_BIT,  // I2C Slave 0
    FIFO_EN_ALL_SENSORS = (FIFO_EN_ACCEL | FIFO_EN_XG | FIFO_EN_YG | FIFO_EN_ZG | FIFO_EN_TEMP)
} FIFO_Source_e;

typedef enum I2C_MasterClock_e {
    I2C_MASTER_CLK_348KHZ = 0x0D, // Recommended for 400kHz operation
    I2C_MASTER_CLK_400KHZ = 0x09, // Alternative
    I2C_MASTER_CLK_300KHZ = 0x0E
} I2C_MasterClock_e;

typedef struct InterruptConfig {
    InterruptPinConfig pinConfig;  /**< Configuration of the physical INT pin */
    uint8_t enabledSources;        /**< Bitmask of enabled interrupt sources (from InterruptSource_e) */
} InterruptConfig;

typedef struct FIFO_Config {
    FIFO_Mode_e mode;              /**< Behavior when the FIFO buffer becomes full */
    uint8_t enabledSources;        /**< Bitmask of sensor data to store in FIFO (from FIFO_Source_e) */
} FIFO_Config;

/**
 * @brief Structure to hold all user-configurable settings for initialization.
 */
typedef struct MPU9250_Config {
    // --- Basic Settings ---
    uint8_t i2cAddress;           /**< I2C address of the sensor (usually 0x68) */
    AccelFSR_e accelFSR;          /**< Accelerometer full-scale range */
    GyroFSR_e gyroFSR;            /**< Gyroscope full-scale range */
    uint8_t sampleRateDivider;    /**< Sample Rate = Gyro/Accel Output Rate / (1 + smplrt_div) */

    // --- Filter Settings ---
    AccelDLPF_e accelDLPF;        /**< Accelerometer Digital Low Pass Filter bandwidth */
    GyroDLPF_e gyroDLPF;          /**< Gyroscope Digital Low Pass Filter bandwidth */

    // --- Magnetometer Settings ---
    MagMode_e magMode;            /**< Magnetometer operating mode */
    MagOutput_e magOutput;        /**< Magnetometer output bit depth */

    // --- Advanced Features ---
    InterruptConfig interrupts;     /**< Interrupt configuration */
    FIFO_Config fifo;             /**< FIFO buffer configuration */
    I2C_MasterClock_e i2cMasterClock; /**< Clock speed for the secondary I2C bus */
    bool enableFsync;             /**< If true, enables the FSYNC pin for external synchronization */
} MPU9250_Config;

/**
 * @brief The main handle for the MPU9250 sensor.
 * @details This structure holds the entire state of a sensor instance, including
 * configuration, calibration data, live sensor readings, and the TI-RTOS I2C handles.
 * A pointer to this structure is passed to all driver functions.
 */
typedef struct MPU9250_Handle {
    /* TI-RTOS Communication Handles */
    I2C_Handle i2cHandle;             /**< Handle to the I2C peripheral from TI-RTOS. */
    I2C_Transaction *i2cTransaction;  /**< Pointer to a user-provided I2C transaction struct. */
    uint8_t i2cAddress;               /**< The I2C address of the sensor. */
    uint8_t pwrGPIO;                  /**< GPIO index to power on or off device */

    /* Sensor State & Data */
    bool hasMag;                      /**< True if an AK8963 magnetometer is detected. */
    MPU9250_Vector3D accelerometer;   /**< Latest accelerometer reading in m/s^2. */
    MPU9250_Vector3D gyroscope;       /**< Latest gyroscope reading in rad/s. */
    MPU9250_Vector3D magnetometer;    /**< Latest magnetometer reading in uT. */
    float temperature;                /**< Latest temperature reading in Celsius. */
    float sampleRate;                 /**< The configured sample rate in Hz. */

    /* Calibration Data */
    MPU9250_Vector3D accelBias;       /**< Accelerometer bias in m/s^2. */
    MPU9250_Vector3D gyroBias;        /**< Gyroscope bias in rad/s. */
    MPU9250_Vector3D magBias;         /**< Magnetometer bias in uT. */
    MPU9250_Vector3D accelVariance;   /**< Accelerometer measurement variance for EKF. */
    MPU9250_Vector3D gyroVariance;    /**< Gyroscope measurement variance for EKF. */
    MPU9250_Vector3D magVariance;     /**< Magnetometer measurement variance for EKF. */

    /* Internal Scale Factors */
    float accelScale;                 /**< Raw to m/s^2 conversion factor. */
    float gyroScale;                  /**< Raw to rad/s conversion factor. */
    float magScale;                   /**< Raw to uT conversion factor. */
    MPU9250_Vector3D magAdjust;       /**< Magnetometer sensitivity adjustment values from fuse ROM. */

    /* Sensor Fusion State */
    MPU9250_FusionState fusion;       /**< Holds the state and results of the fusion algorithm. */
} MPU9250_Handle;

// --- Function Prototypes ---
/**
 * @brief Fills a configuration structure with default recommended settings.
 * @param[out] config Pointer to the configuration structure to be filled.
 */
void MPU9250_getDefaultConfig(MPU9250_Config *config);

/**
 * @brief Initializes the MPU9250 sensor.
 * @details This function allocates memory for the sensor handle, performs a device reset,
 * checks the WHO_AM_I register to identify the sensor variant (MPU9250/6500),
 * and configures the accelerometer, gyroscope, and magnetometer according to the
 * provided configuration structure.
 * @param[out] sensor_p Pointer to the handle pointer that will be allocated and initialized.
 * @param[in]  i2c Handle to the TI-RTOS I2C peripheral.
 * @param[in]  transaction Pointer to a user-provided I2C transaction structure.
 * @param[in]  config Pointer to a configuration structure with the desired settings.
 * @param[in]  pwrGPIO GPIO index used to power on or off the device.
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_init(MPU9250_Handle **sensor_p, const I2C_Handle i2c, const I2C_Transaction *transaction, const MPU9250_Config *config, const uint8_t pwrGPIO);

/**
 * @brief Deinitializes the sensor and frees the handle's memory.
 * @param[in] sensor Pointer to the sensor handle to deinitialize.
 */
void MPU9250_deinit(MPU9250_Handle *sensor);

/**
 * @brief Reads the latest data from all sensors.
 * @details Performs a burst read from the sensor to get accelerometer, gyroscope,
 * and temperature data. If a magnetometer is present, it also reads its data.
 * The raw values are then converted to engineering units (m/s^2, rad/s, uT)
 * using the scale factors and biases stored in the handle. The results are
 * stored back into the `accelerometer`, `gyroscope`, `magnetometer`, and
 * `temperature` fields of the handle.
 * @param[in,out] sensor Pointer to the sensor handle.
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_readSensors(MPU9250_Handle *sensor);

/**
 * @brief Performs a static calibration of the IMU.
 * @details This function measures the average sensor readings over a number of samples
 * while the device is held stationary. It calculates the biases (offsets) and
 * variances for the accelerometer, gyroscope, and magnetometer. These values are
 * crucial for accurate sensor fusion. The calculated values are stored in the
 * `accelBias`, `gyroBias`, `magBias`, and variance fields of the handle.
 * @pre The device must be kept perfectly still during the entire calibration process.
 * @param[in,out] sensor Pointer to the sensor handle.
 * @param[in]     numSamples The number of samples to average for the calibration (e.g., 1000).
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_calibrate(MPU9250_Handle *sensor, const uint16_t numSamples);

/**
 * @brief Runs an interactive routine to determine the sensor's orientation matrix.
 * @details This function guides the user through a two-step process to find the
 * physical orientation of the sensor relative to a North-East-Down (NED) frame.
 * 1. Finds the DOWN vector by measuring gravity while the device is level.
 * 2. Finds the NORTH (forward) vector by detecting a positive roll motion.
 * It then calculates the corresponding rotation matrix that can be used to
 * transform sensor readings from the body frame to the NED frame.
 * @param[in]  sensor Pointer to the initialized and calibrated sensor handle.
 * @param[out] matrix Pointer to a 3x3 matrix where the calculated orientation matrix will be stored.
 * @param[in]  frame The desired reference frame (FRAME_NED or FRAME_ENU).
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_determineOrientationAxes(const MPU9250_Handle *sensor, MPU9250_Matrix3x3 *matrix, const ReferenceFrame_e frame);

/**
 * @brief Applies a rotation to a 3D vector using a rotation matrix.
 * @details This is a utility function to transform a vector from one coordinate
 * frame to another (e.g., from the sensor's body frame to the world/NED frame).
 * `V_rotated = R * V_original`.
 * @param[in]  matrix Pointer to the 3x3 rotation matrix.
 * @param[in,out] vector Pointer to the vector to be rotated. It will be modified in-place.
 */
void MPU9250_applyRotation(const MPU9250_Matrix3x3 *matrix, MPU9250_Vector3D *vector);

/**
 * @brief Initializes the sensor fusion module.
 * @details This function configures the internal sensor fusion state based on the
 * selected algorithm and its specific parameters. It also resets the orientation
 * to a known state (a unit quaternion).
 * @param[in,out] sensor Pointer to the sensor handle.
 * @param[in]     algorithm The fusion algorithm to use.
 * @param[in]     params A void pointer to a parameter struct corresponding to the
 * chosen algorithm (e.g., a `MadgwickFilterParams` struct).
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_SensorFusionInit(MPU9250_Handle *sensor, const FusionAlgorithm_e algorithm, const void *params);

/**
 * @brief Updates the sensor fusion algorithm with the latest sensor data.
 * @details This function takes the latest accelerometer, gyroscope, and (optionally)
 * magnetometer data from the sensor handle and runs one iteration of the chosen
 * fusion algorithm. The resulting orientation is stored internally in the handle.
 * @pre MPU9250_readSensors() should be called before this function to update the sensor data.
 * @pre For accurate results, the sensor data should be rotated to the NED frame
 * using `MPU9250_applyRotation()` before calling this function.
 * @param[in,out] sensor Pointer to the sensor handle.
 * @param[in]     dof The degrees of freedom to use for this update (6-DoF or 9-DoF).
 * @param[in]     dt The time elapsed since the last update, in seconds.
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_SensorFusionUpdate(MPU9250_Handle *sensor, const FusionDOF_e dof, const float dt);

/**
 * @brief Gets the current orientation as a quaternion.
 * @param[in]  sensor Pointer to the sensor handle.
 * @param[out] q Pointer to a quaternion structure to store the result.
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_getOrientation(const MPU9250_Handle *sensor, MPU9250_Quaternion *q);

/**
 * @brief Gets the current orientation as Euler angles.
 * @details Converts the internal orientation quaternion to Euler angles (roll, pitch, yaw)
 * in radians.
 * @param[in]  sensor Pointer to the sensor handle.
 * @param[out] angles Pointer to an EulerAngles structure to store the result in radians.
 * @return MPU9250_Status_e Status code indicating the result of the operation.
 */
MPU9250_Status_e MPU9250_getEulerAngles(const MPU9250_Handle *sensor, MPU9250_EulerAngles *angles);

#endif // MPU9250_H
