#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BMI160_I2C_ADDR	(0x69)

typedef enum {
    BMI160_ACCEL_RATE_25_2HZ = 5,  /**<   25/2  Hz */
    BMI160_ACCEL_RATE_25HZ,        /**<   25    Hz */
    BMI160_ACCEL_RATE_50HZ,        /**<   50    Hz */
    BMI160_ACCEL_RATE_100HZ,       /**<  100    Hz */
    BMI160_ACCEL_RATE_200HZ,       /**<  200    Hz */
    BMI160_ACCEL_RATE_400HZ,       /**<  400    Hz */
    BMI160_ACCEL_RATE_800HZ,       /**<  800    Hz */
    BMI160_ACCEL_RATE_1600HZ,      /**< 1600    Hz */
} BMI160AccelRate;

typedef enum {
    BMI160_GYRO_RATE_25HZ = 6,     /**<   25    Hz */
    BMI160_GYRO_RATE_50HZ,         /**<   50    Hz */
    BMI160_GYRO_RATE_100HZ,        /**<  100    Hz */
    BMI160_GYRO_RATE_200HZ,        /**<  200    Hz */
    BMI160_GYRO_RATE_400HZ,        /**<  400    Hz */
    BMI160_GYRO_RATE_800HZ,        /**<  800    Hz */
    BMI160_GYRO_RATE_1600HZ,       /**< 1600    Hz */
    BMI160_GYRO_RATE_3200HZ,       /**< 3200    Hz */
} BMI160GyroRate;

typedef enum {
    BMI160_DLPF_MODE_NORM = 0x2,
    BMI160_DLPF_MODE_OSR2 = 0x1,
    BMI160_DLPF_MODE_OSR4 = 0x0,
} BMI160DLPFMode;

typedef enum {
    BMI160_ACCEL_RANGE_2G  = 0X03, /**<  +/-  2g range */
    BMI160_ACCEL_RANGE_4G  = 0X05, /**<  +/-  4g range */
    BMI160_ACCEL_RANGE_8G  = 0X08, /**<  +/-  8g range */
    BMI160_ACCEL_RANGE_16G = 0X0C, /**<  +/- 16g range */
} BMI160AccelRange;

typedef enum {
    BMI160_GYRO_RANGE_2000 = 0, /**<  +/- 2000 degrees/second */
    BMI160_GYRO_RANGE_1000,     /**<  +/- 1000 degrees/second */
    BMI160_GYRO_RANGE_500,      /**<  +/-  500 degrees/second */
    BMI160_GYRO_RANGE_250,      /**<  +/-  250 degrees/second */
    BMI160_GYRO_RANGE_125,      /**<  +/-  125 degrees/second */
} BMI160GyroRange;

typedef enum {
    BMI160_STEP_MODE_NORMAL = 0,
    BMI160_STEP_MODE_SENSITIVE,
    BMI160_STEP_MODE_ROBUST,
    BMI160_STEP_MODE_UNKNOWN,
} BMI160StepMode;

typedef enum {
    BMI160_TAP_SHOCK_DURATION_50MS = 0,
    BMI160_TAP_SHOCK_DURATION_75MS,
} BMI160TapShockDuration;

typedef enum {
    BMI160_TAP_QUIET_DURATION_30MS = 0,
    BMI160_TAP_QUIET_DURATION_20MS,
} BMI160TapQuietDuration;

typedef enum {
    BMI160_DOUBLE_TAP_DURATION_50MS = 0,
    BMI160_DOUBLE_TAP_DURATION_100MS,
    BMI160_DOUBLE_TAP_DURATION_150MS,
    BMI160_DOUBLE_TAP_DURATION_200MS,
    BMI160_DOUBLE_TAP_DURATION_250MS,
    BMI160_DOUBLE_TAP_DURATION_375MS,
    BMI160_DOUBLE_TAP_DURATION_500MS,
    BMI160_DOUBLE_TAP_DURATION_700MS,
} BMI160DoubleTapDuration;

typedef enum {
    BMI160_ZERO_MOTION_DURATION_1_28S   = 0x00, /**<   1.28 seconds */
    BMI160_ZERO_MOTION_DURATION_2_56S,          /**<   2.56 seconds */
    BMI160_ZERO_MOTION_DURATION_3_84S,          /**<   3.84 seconds */
    BMI160_ZERO_MOTION_DURATION_5_12S,          /**<   5.12 seconds */
    BMI160_ZERO_MOTION_DURATION_6_40S,          /**<   6.40 seconds */
    BMI160_ZERO_MOTION_DURATION_7_68S,          /**<   7.68 seconds */
    BMI160_ZERO_MOTION_DURATION_8_96S,          /**<   8.96 seconds */
    BMI160_ZERO_MOTION_DURATION_10_24S,         /**<  10.24 seconds */
    BMI160_ZERO_MOTION_DURATION_11_52S,         /**<  11.52 seconds */
    BMI160_ZERO_MOTION_DURATION_12_80S,         /**<  12.80 seconds */
    BMI160_ZERO_MOTION_DURATION_14_08S,         /**<  14.08 seconds */
    BMI160_ZERO_MOTION_DURATION_15_36S,         /**<  15.36 seconds */
    BMI160_ZERO_MOTION_DURATION_16_64S,         /**<  16.64 seconds */
    BMI160_ZERO_MOTION_DURATION_17_92S,         /**<  17.92 seconds */
    BMI160_ZERO_MOTION_DURATION_19_20S,         /**<  19.20 seconds */
    BMI160_ZERO_MOTION_DURATION_20_48S,         /**<  20.48 seconds */
    BMI160_ZERO_MOTION_DURATION_25_60S  = 0x10, /**<  25.60 seconds */
    BMI160_ZERO_MOTION_DURATION_30_72S,         /**<  30.72 seconds */
    BMI160_ZERO_MOTION_DURATION_35_84S,         /**<  35.84 seconds */
    BMI160_ZERO_MOTION_DURATION_40_96S,         /**<  40.96 seconds */
    BMI160_ZERO_MOTION_DURATION_46_08S,         /**<  46.08 seconds */
    BMI160_ZERO_MOTION_DURATION_51_20S,         /**<  51.20 seconds */
    BMI160_ZERO_MOTION_DURATION_56_32S,         /**<  56.32 seconds */
    BMI160_ZERO_MOTION_DURATION_61_44S,         /**<  61.44 seconds */
    BMI160_ZERO_MOTION_DURATION_66_56S,         /**<  66.56 seconds */
    BMI160_ZERO_MOTION_DURATION_71_68S,         /**<  71.68 seconds */
    BMI160_ZERO_MOTION_DURATION_76_80S,         /**<  76.80 seconds */
    BMI160_ZERO_MOTION_DURATION_81_92S,         /**<  81.92 seconds */
    BMI160_ZERO_MOTION_DURATION_87_04S,         /**<  87.04 seconds */
    BMI160_ZERO_MOTION_DURATION_92_16S,         /**<  92.16 seconds */
    BMI160_ZERO_MOTION_DURATION_97_28S,         /**<  97.28 seconds */
    BMI160_ZERO_MOTION_DURATION_102_40S,        /**< 102.40 seconds */
    BMI160_ZERO_MOTION_DURATION_112_64S = 0x20, /**< 112.64 seconds */
    BMI160_ZERO_MOTION_DURATION_122_88S,        /**< 122.88 seconds */
    BMI160_ZERO_MOTION_DURATION_133_12S,        /**< 133.12 seconds */
    BMI160_ZERO_MOTION_DURATION_143_36S,        /**< 143.36 seconds */
    BMI160_ZERO_MOTION_DURATION_153_60S,        /**< 153.60 seconds */
    BMI160_ZERO_MOTION_DURATION_163_84S,        /**< 163.84 seconds */
    BMI160_ZERO_MOTION_DURATION_174_08S,        /**< 174.08 seconds */
    BMI160_ZERO_MOTION_DURATION_184_32S,        /**< 184.32 seconds */
    BMI160_ZERO_MOTION_DURATION_194_56S,        /**< 194.56 seconds */
    BMI160_ZERO_MOTION_DURATION_204_80S,        /**< 204.80 seconds */
    BMI160_ZERO_MOTION_DURATION_215_04S,        /**< 215.04 seconds */
    BMI160_ZERO_MOTION_DURATION_225_28S,        /**< 225.28 seconds */
    BMI160_ZERO_MOTION_DURATION_235_52S,        /**< 235.52 seconds */
    BMI160_ZERO_MOTION_DURATION_245_76S,        /**< 245.76 seconds */
    BMI160_ZERO_MOTION_DURATION_256_00S,        /**< 256.00 seconds */
    BMI160_ZERO_MOTION_DURATION_266_24S,        /**< 266.24 seconds */
    BMI160_ZERO_MOTION_DURATION_276_48S,        /**< 276.48 seconds */
    BMI160_ZERO_MOTION_DURATION_286_72S,        /**< 286.72 seconds */
    BMI160_ZERO_MOTION_DURATION_296_96S,        /**< 296.96 seconds */
    BMI160_ZERO_MOTION_DURATION_307_20S,        /**< 307.20 seconds */
    BMI160_ZERO_MOTION_DURATION_317_44S,        /**< 317.44 seconds */
    BMI160_ZERO_MOTION_DURATION_327_68S,        /**< 327.68 seconds */
    BMI160_ZERO_MOTION_DURATION_337_92S,        /**< 337.92 seconds */
    BMI160_ZERO_MOTION_DURATION_348_16S,        /**< 348.16 seconds */
    BMI160_ZERO_MOTION_DURATION_358_40S,        /**< 358.40 seconds */
    BMI160_ZERO_MOTION_DURATION_368_64S,        /**< 368.64 seconds */
    BMI160_ZERO_MOTION_DURATION_378_88S,        /**< 378.88 seconds */
    BMI160_ZERO_MOTION_DURATION_389_12S,        /**< 389.12 seconds */
    BMI160_ZERO_MOTION_DURATION_399_36S,        /**< 399.36 seconds */
    BMI160_ZERO_MOTION_DURATION_409_60S,        /**< 409.60 seconds */
    BMI160_ZERO_MOTION_DURATION_419_84S,        /**< 419.84 seconds */
    BMI160_ZERO_MOTION_DURATION_430_08S,        /**< 430.08 seconds */
} BMI160ZeroMotionDuration;

typedef struct {
	float x;
	float y;
	float z;
} GyroAccelData;

typedef struct {
	float AngleRoll;
	float AnglePitch;
} AnglePeachData;

typedef struct {
    int (*transmit)(uint16_t address,  uint8_t *data, uint16_t size);
    int (*receive)(uint16_t address,  uint8_t *data, uint16_t size);
    int (*delayMs)(uint32_t time);
} I2C_Callbacks;

typedef struct {
	BMI160AccelRange accelRange;
	BMI160AccelRate accelRate;
	BMI160DLPFMode accelmode;
	BMI160GyroRange gyroRange;
	BMI160GyroRate gyroRate;
	BMI160DLPFMode gyroMode;
} BMI160_Config;

class BMI160 {
  public:

	BMI160(uint16_t i2cAddress, I2C_Callbacks *callbacks);

	int init(BMI160_Config *config);

	uint8_t readChipID(void);
	int softReset(void);
	uint8_t checkSensorID(void);
	int resetStepCount(void);
	uint16_t getStepCount();
	uint8_t getStepDetectionMode(void);
	int setStepDetectionMode(BMI160StepMode mode);
	int setStepCountEnabled(uint8_t enabled);
	uint8_t getStepCountEnabled(void);

	int powerUpAccelerometer(void);

	BMI160DLPFMode getAccelDLPFMode(void);
	int setAccelDLPFMode(BMI160DLPFMode mode);

	BMI160GyroRange getFullScaleGyroRange(void);
	int setFullScaleAccelRange(BMI160AccelRange range);

	uint8_t getAccelOffsetEnabled(void);
	int setAccelOffsetEnabled(const uint8_t enabled);

	int readAccelerometer(int16_t* x, int16_t* y, int16_t* z);
	GyroAccelData scaledAccelData();
	AnglePeachData getAccelerometerAngles();

	BMI160AccelRate getAccelRate(void);
	int setAccelRate(const BMI160AccelRate rate);

	int autoCalibrateXAccelOffset(int target);
	int autoCalibrateYAccelOffset(int target);
	int autoCalibrateZAccelOffset(int target);

	int8_t getXAccelOffset(void);
	int setXAccelOffset(int8_t offset);
	int8_t getYAccelOffset();
	int setYAccelOffset(int8_t offset);
	int8_t getZAccelOffset();
	int setZAccelOffset(int8_t offset);

	int powerUpGyroscope(void);
	BMI160DLPFMode getGyroDLPFMode(void);
	int setGyroDLPFMode(BMI160DLPFMode mode);

	BMI160AccelRange getFullScaleAccelRange(void);
	int setFullScaleGyroRange(BMI160GyroRange range);

	uint8_t getGyroOffsetEnabled(void);
	int setGyroOffsetEnabled(uint8_t enabled);


	int readGyro(int16_t* x, int16_t* y, int16_t* z);
	GyroAccelData scaledGyroData();

	BMI160GyroRate getGyroRate(void);
	int setGyroRate(const BMI160GyroRate rate);

	int autoCalibrateGyroOffset(void);

	int16_t getXGyroOffset(void);
	int setXGyroOffset(int16_t offset);
	int16_t getYGyroOffset(void);
	int setYGyroOffset(int16_t offset);
	int16_t getZGyroOffset(void);
	int setZGyroOffset(int16_t offset);

	int readTemperature(int16_t *t);
	float scaledTemperature();

	int readMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

	uint8_t getFreefallDetectionThreshold(void);
	int setFreefallDetectionThreshold(uint8_t threshold);
	uint8_t getFreefallDetectionDuration(void);
	int setFreefallDetectionDuration(uint8_t duration);
	uint8_t getShockDetectionThreshold(void);
	int setShockDetectionThreshold(uint8_t threshold);
	uint8_t getShockDetectionDuration(void);
	int setShockDetectionDuration(uint8_t duration);

	uint8_t getMotionDetectionThreshold(void);
	int setMotionDetectionThreshold(uint8_t threshold);
	uint8_t getMotionDetectionDuration(void);
	int setMotionDetectionDuration(uint8_t samples);
	uint8_t getZeroMotionDetectionThreshold(void);
	int setZeroMotionDetectionThreshold(const uint8_t threshold);
	uint8_t getZeroMotionDetectionDuration(void);
	int setZeroMotionDetectionDuration(const uint8_t duration);
	uint8_t getTapDetectionThreshold(void);
	int setTapDetectionThreshold(const uint8_t threshold);
	uint8_t getTapShockDuration(void);

  private:
	uint16_t i2cAddress;
	I2C_Callbacks callbacks;

	BMI160GyroRange gyroRange;
	BMI160AccelRange accelRange;

	int writeRegister(uint8_t address, uint8_t cmd);
	int readRegister(uint8_t address, uint8_t dataSize, uint8_t *rec);
	int writeRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len);
	uint8_t readRegBits(uint8_t reg, unsigned pos, unsigned len);
};

