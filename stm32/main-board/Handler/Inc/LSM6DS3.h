////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      LSM6DS3.h
//!  \brief     Inertial sensor IC register definitions
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

// Defines -------------------------------------------------------------------------------------------------------------

#define LSM6DS3_ADDR0          0xD4 /* SD0 on GND */
#define LSM6DS3_ADDR1          0xD6 /* SD0 on VDD */

// Embedded functions configuration register ---------------
#define FUNC_CFG_ACCESS        0x01

// Sensor sync configuration register ----------------------
#define SENSOR_SYNC_TIME_FRAME 0x04

// FIFO configuration registers ----------------------------
#define FIFO_CTRL1             0x06
#define FIFO_CTRL2             0x07
#define FIFO_CTRL3             0x08
#define FIFO_CTRL4             0x09
#define FIFO_CTRL5             0x0A

#define ORIENT_CFG_G           0x0B

// INT pin control -----------------------------------------
#define INT1_CTRL              0x0D
#define INT2_CTRL              0x0E

// Who I am ID ---------------------------------------------
#define WHO_AM_I               0x0F

// Accelerometer and gyroscope control registers -----------
#define CTRL1_XL               0x10
#define CTRL2_G                0x11
#define CTRL3_C                0x12
#define CTRL4_C                0x13
#define CTRL5_C                0x14
#define CTRL6_C                0x15
#define CTRL7_G                0x16
#define CTRL8_XL               0x17
#define CTRL9_XL               0x18
#define CTRL10_C               0x19

// I2C master configuration register -----------------------
#define MASTER_CONFIG          0x1A

// Interrupts registers ------------------------------------
#define WAKE_UP_SRC            0x1B
#define TAP_SRC                0x1C
#define D6D_SRC                0x1D

// Status data register ------------------------------------
#define STATUS_REG             0x1E

// Temperature output data register ------------------------
#define OUT_TEMP_L             0x20
#define OUT_TEMP_H             0x21

// Gyroscope output register -------------------------------
#define OUTX_L_G               0x22
#define OUTX_H_G               0x23
#define OUTY_L_G               0x24
#define OUTY_H_G               0x25
#define OUTZ_L_G               0x26
#define OUTZ_H_G               0x27

// Accelerometer output register ---------------------------
#define OUTX_L_XL              0x28
#define OUTX_H_XL              0x29
#define OUTY_L_XL              0x2A
#define OUTY_H_XL              0x2B
#define OUTZ_L_XL              0x2C
#define OUTZ_H_XL              0x2D

// Sensor hub output registers -----------------------------
#define SENSORHUB1_REG         0x2E
#define SENSORHUB2_REG         0x2F
#define SENSORHUB3_REG         0x30
#define SENSORHUB4_REG         0x31
#define SENSORHUB5_REG         0x32
#define SENSORHUB6_REG         0x33
#define SENSORHUB7_REG         0x34
#define SENSORHUB8_REG         0x35
#define SENSORHUB9_REG         0x36
#define SENSORHUB10_REG        0x37
#define SENSORHUB11_REG        0x38
#define SENSORHUB12_REG        0x39

// FIFO status registers -----------------------------------
#define FIFO_STATUS1           0x3A
#define FIFO_STATUS2           0x3B
#define FIFO_STATUS3           0x3C
#define FIFO_STATUS4           0x3D

// FIFO data output registers ------------------------------
#define FIFO_DATA_OUT_L        0x3E
#define FIFO_DATA_OUT_H        0x3F

// Timestamp output registers ------------------------------
#define TIMESTAMP0_REG         0x40
#define TIMESTAMP1_REG         0x41
#define TIMESTAMP2_REG         0x42

// Step counter timestamp registers ------------------------
#define STEP_TIMESTAMP_L       0x49
#define STEP_TIMESTAMP_H       0x4A

// Step counter output registers ---------------------------
#define STEP_COUNTER_L         0x4B
#define STEP_COUNTER_H         0x4C

// Sensor hub output registers -----------------------------
#define SENSORHUB13_REG        0x4D
#define SENSORHUB14_REG        0x4E
#define SENSORHUB15_REG        0x4F
#define SENSORHUB16_REG        0x50
#define SENSORHUB17_REG        0x51
#define SENSORHUB18_REG        0x52

// Interrupt register --------------------------------------
#define FUNC_SRC               0x53

// Interrupt registers
#define TAP_CFG                0x58
#define TAP_THS_6D             0x59
#define INT_DUR2               0x5A
#define WAKE_UP_THS            0x5B
#define WAKE_UP_DUR            0x5C
#define FREE_FALL              0x5D
#define MD1_CFG                0x5E
#define MD2_CFG                0x5F

// External magnetometer raw data output registers ---------
#define OUT_MAG_RAW_X_L        0x66
#define OUT_MAG_RAW_X_H        0x67
#define OUT_MAG_RAW_Y_L        0x68
#define OUT_MAG_RAW_Y_H        0x69
#define OUT_MAG_RAW_Z_L        0x6A
#define OUT_MAG_RAW_Z_H        0x6B

// Embedded function registers -----------------------------

#define SLV0_ADD                    0x02
#define SLV0_SUBADD                 0x03
#define SLAVE0_CONFIG               0x04
#define SLV1_ADD                    0x05
#define SLV1_SUBADD                 0x06
#define SLAVE1_CONFIG               0x07
#define SLV2_ADD                    0x08
#define SLV2_SUBADD                 0x09
#define SLAVE2_CONFIG               0x0A
#define SLV3_ADD                    0x0B
#define SLV3_SUBADD                 0x0C
#define SLAVE3_CONFIG               0x0D
#define DATAWRITE_SRC_MODE_SUB_SLV0 0x0E
#define PEDO_THS_REG                0x0F
#define SM_THS                      0x13
#define PEDO_DEB_REG                0x14
#define STEP_COUNT_DELTA            0x15
#define MAG_SI_XX                   0x24
#define MAG_SI_XY                   0x25
#define MAG_SI_XZ                   0x26
#define MAG_SI_YX                   0x27
#define MAG_SI_YY                   0x28
#define MAG_SI_YZ                   0x29
#define MAG_SI_ZX                   0x2A
#define MAG_SI_ZY                   0x2B
#define MAG_SI_ZZ                   0x2C
#define MAG_OFFX_L                  0x2D
#define MAG_OFFX_H                  0x2E
#define MAG_OFFY_L                  0x2F
#define MAG_OFFY_H                  0x30
#define MAG_OFFZ_L                  0x31
#define MAG_OFFZ_H                  0x32

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------
