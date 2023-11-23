#include <BMI160.h>
#include "BMI160_defs.h"
#include <math.h>

#define BMI160_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)

BMI160::BMI160(uint16_t i2cAddress, I2C_Callbacks *callbacks) {
	this->i2cAddress = i2cAddress;
	this->callbacks = *callbacks;
}

int BMI160::init(BMI160_Config *config) {

	int opStatus = softReset();
	if(opStatus != 0) {
		return opStatus;
	}

	callbacks.delayMs(2);

	if(checkSensorID() == 0) { return -1; }

	powerUpAccelerometer();
	setFullScaleAccelRange(config->accelRange);
	setAccelRate(config->accelRate);
	setAccelDLPFMode(config->accelmode);
	setAccelOffsetEnabled(1);

	powerUpGyroscope();
	setFullScaleGyroRange(config->gyroRange);
	setGyroRate(config->gyroRate);
	setGyroDLPFMode(config->gyroMode);
	setGyroOffsetEnabled(1);

	return 0;
}

uint8_t BMI160::readChipID(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_CHIP_ID, 1, &readData);
	return readData;
}

int BMI160::softReset(void) {
	return writeRegister(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
}

int BMI160::powerUpAccelerometer(void) {

	int opStatus = writeRegister(BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
	if(opStatus != 0) {
		return opStatus;
	}

	callbacks.delayMs(1);
	while (0x1 != readRegBits(BMI160_RA_PMU_STATUS, BMI160_ACC_PMU_STATUS_BIT, BMI160_ACC_PMU_STATUS_LEN)) {
		callbacks.delayMs(1);
	}

	return 0;
}

int BMI160::powerUpGyroscope(void) {

	int opStatus = writeRegister(BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
	if(opStatus != 0) {
		return opStatus;
	}

	callbacks.delayMs(1);
	while (0x1 != readRegBits(BMI160_RA_PMU_STATUS, BMI160_GYR_PMU_STATUS_BIT, BMI160_GYR_PMU_STATUS_LEN)) {
		callbacks.delayMs(1);
	}

	return 0;
}

uint8_t BMI160::checkSensorID(void) {

	uint8_t readedChipId = readChipID();
	if(readedChipId == SENSOR_CHIP_ID_BMI160) {
		return 1;
	}
	else if(readedChipId == SENSOR_CHIP_ID_BMI160_C2) {
		return 2;
	}
	else if(readedChipId == SENSOR_CHIP_ID_BMI160_C3) {
		return 3;
	}

	return 0;
}

int BMI160::readGyro(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t buffer[6];

    buffer[0] = BMI160_RA_GYRO_X_L;

    int opStat = readRegister(BMI160_RA_GYRO_X_L, 6, &buffer[0]);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];

    return opStat;
}

GyroAccelData BMI160::scaledAccelData() {
	GyroAccelData result = {};

	int16_t x, y, z = 0;
	readAccelerometer(&x, &y, &z);

    if(accelRange == BMI160_ACCEL_RANGE_2G) {
    	result.x = (x / SENS_2G_MS2_LSB_PER_MS2);
    	result.y = (y / SENS_2G_MS2_LSB_PER_MS2);
    	result.z = (z / SENS_2G_MS2_LSB_PER_MS2);
    }
    else if(accelRange == BMI160_ACCEL_RANGE_4G) {
    	result.x = (x / SENS_4G_MS2_LSB_PER_MS2);
    	result.y = (y / SENS_4G_MS2_LSB_PER_MS2);
    	result.z = (z / SENS_4G_MS2_LSB_PER_MS2);
    }
    else if(accelRange == BMI160_ACCEL_RANGE_8G) {
    	result.x = (x / SENS_8G_MS2_LSB_PER_MS2);
    	result.y = (y / SENS_8G_MS2_LSB_PER_MS2);
    	result.z = (z / SENS_8G_MS2_LSB_PER_MS2);
    }
    else if(accelRange == BMI160_ACCEL_RANGE_16G) {
    	result.x = (x / SENS_16G_MS2_LSB_PER_MS2);
    	result.y = (y / SENS_16G_MS2_LSB_PER_MS2);
    	result.z = (z / SENS_16G_MS2_LSB_PER_MS2);
    }

    return result;
}

AnglePeachData BMI160::getAccelerometerAngles() {
	AnglePeachData result = {};

	int16_t x, y, z = 0;
	readAccelerometer(&x, &y, &z);

	result.AngleRoll = atan2(y, sqrt((int32_t)x*x + (int32_t)z*z)) * 57.288;
	result.AnglePitch = -1 * atan2(x, sqrt((int32_t)y*y+ (int32_t)z*z)) * 57.288;

	return result;
}

GyroAccelData BMI160::scaledGyroData() {
	GyroAccelData result = {};

	int16_t x, y, z = 0;
	readGyro(&x, &y, &z);

    if(gyroRange == BMI160_GYRO_RANGE_2000) {
    	result.x = (x / SENS_2000_DPS_LSB_PER_DPS);
    	result.y = (y / SENS_2000_DPS_LSB_PER_DPS);
    	result.z = (z / SENS_2000_DPS_LSB_PER_DPS);
    }
    else if(gyroRange == BMI160_GYRO_RANGE_1000) {
    	result.x = (x / SENS_1000_DPS_LSB_PER_DPS);
    	result.y = (y / SENS_1000_DPS_LSB_PER_DPS);
    	result.z = (z / SENS_1000_DPS_LSB_PER_DPS);
    }
    else if(gyroRange == BMI160_GYRO_RANGE_500) {
    	result.x = (x / SENS_500_DPS_LSB_PER_DPS);
    	result.y = (y / SENS_500_DPS_LSB_PER_DPS);
    	result.z = (z / SENS_500_DPS_LSB_PER_DPS);
    }
    else if(gyroRange == BMI160_GYRO_RANGE_250) {
    	result.x = (x / SENS_250_DPS_LSB_PER_DPS);
    	result.y = (y / SENS_250_DPS_LSB_PER_DPS);
    	result.z = (z / SENS_250_DPS_LSB_PER_DPS);
    }
    else if(gyroRange == BMI160_GYRO_RANGE_125) {
    	result.x = (x / SENS_125_DPS_LSB_PER_DPS);
    	result.y = (y / SENS_125_DPS_LSB_PER_DPS);
    	result.z = (z / SENS_125_DPS_LSB_PER_DPS);
    }

    return result;
}

int BMI160::readTemperature(int16_t *t) {

	uint8_t buffer[2] = {0x00};
    int opStat = readRegister(BMI160_RA_TEMP_L, 2, &buffer[0]);

    if(opStat != 0) {
    	return opStat;
    }

    *t = (((int16_t)buffer[1]) << 8) | buffer[0];
    return opStat;
}

float BMI160::scaledTemperature() {

	int16_t tempRaw = 0;
	readTemperature(&tempRaw);

	float convertTemp = 0;

    if(tempRaw & 0x8000) { convertTemp = (23.0F - ((0x10000 - tempRaw)/512.0F)); }
    else { convertTemp = ((tempRaw/512.0F) + 23.0F); }

    return convertTemp;
}

int BMI160::readAccelerometer(int16_t* x, int16_t* y, int16_t* z) {

    uint8_t buffer[6] = {0x00};
    int opStat = readRegister(BMI160_RA_ACCEL_X_L, 6, &buffer[0]);

    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];

    return opStat;
}

int BMI160::readMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[12] = {0x00};

    int opStat = readRegister(BMI160_RA_GYRO_X_L, 12, &buffer[0]);

    *gx = (((int16_t)buffer[1])  << 8) | buffer[0];
    *gy = (((int16_t)buffer[3])  << 8) | buffer[2];
    *gz = (((int16_t)buffer[5])  << 8) | buffer[4];
    *ax = (((int16_t)buffer[7])  << 8) | buffer[6];
    *ay = (((int16_t)buffer[9])  << 8) | buffer[8];
    *az = (((int16_t)buffer[11]) << 8) | buffer[10];

    return opStat;
}

BMI160GyroRate BMI160::getGyroRate(void) {
	return (BMI160GyroRate)readRegBits(BMI160_RA_GYRO_CONF, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

int BMI160::setGyroRate(const BMI160GyroRate rate) {
	return writeRegisterBits(BMI160_RA_GYRO_CONF, (uint8_t)rate, BMI160_GYRO_RATE_SEL_BIT, BMI160_GYRO_RATE_SEL_LEN);
}

BMI160AccelRate BMI160::getAccelRate(void) {
	return (BMI160AccelRate)readRegBits(BMI160_RA_ACCEL_CONF, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

int BMI160::setAccelRate(const BMI160AccelRate rate) {
	return writeRegisterBits(BMI160_RA_ACCEL_CONF, (uint8_t)rate, BMI160_ACCEL_RATE_SEL_BIT, BMI160_ACCEL_RATE_SEL_LEN);
}

BMI160DLPFMode BMI160::getGyroDLPFMode(void) {
    return (BMI160DLPFMode)readRegBits(BMI160_RA_GYRO_CONF, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

int BMI160::setGyroDLPFMode(BMI160DLPFMode mode) {
	return writeRegisterBits(BMI160_RA_GYRO_CONF, mode, BMI160_GYRO_DLPF_SEL_BIT, BMI160_GYRO_DLPF_SEL_LEN);
}

BMI160DLPFMode BMI160::getAccelDLPFMode(void) {
    return (BMI160DLPFMode)readRegBits(BMI160_RA_ACCEL_CONF, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

int BMI160::setAccelDLPFMode(BMI160DLPFMode mode) {
    return writeRegisterBits(BMI160_RA_ACCEL_CONF, mode, BMI160_ACCEL_DLPF_SEL_BIT, BMI160_ACCEL_DLPF_SEL_LEN);
}

BMI160GyroRange BMI160::getFullScaleGyroRange(void) {
	return (BMI160GyroRange)readRegBits(BMI160_RA_GYRO_RANGE, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

int BMI160::setFullScaleGyroRange(const BMI160GyroRange range) {
	gyroRange = range;
    return writeRegisterBits(BMI160_RA_GYRO_RANGE, (uint8_t)range, BMI160_GYRO_RANGE_SEL_BIT, BMI160_GYRO_RANGE_SEL_LEN);
}

BMI160AccelRange BMI160::getFullScaleAccelRange(void) {
	return (BMI160AccelRange)readRegBits(BMI160_RA_ACCEL_RANGE, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

int BMI160::setFullScaleAccelRange(const BMI160AccelRange range) {
	accelRange = range;
	return writeRegisterBits(BMI160_RA_ACCEL_RANGE, (uint8_t)range, BMI160_ACCEL_RANGE_SEL_BIT, BMI160_ACCEL_RANGE_SEL_LEN);
}

uint8_t BMI160::getAccelOffsetEnabled(void) {
    return !!(readRegBits(BMI160_RA_OFFSET_6, BMI160_ACC_OFFSET_EN, 1));
}

int BMI160::setAccelOffsetEnabled(const uint8_t enabled) {
	return writeRegisterBits(BMI160_RA_OFFSET_6, enabled ? 0x1 : 0, BMI160_ACC_OFFSET_EN, 1);
}

int BMI160::autoCalibrateXAccelOffset(int target) {
    uint8_t foc_conf = 0;
    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_X_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_X_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_X_BIT); }
    else { return -1; }

    writeRegister(BMI160_RA_FOC_CONF, foc_conf);
    writeRegister(BMI160_RA_CMD, BMI160_CMD_START_FOC);
    while (!(readRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) {
    	callbacks.delayMs(1);
    }

    return 0;
}

int BMI160::autoCalibrateYAccelOffset(int target) {
    uint8_t foc_conf = 0;
    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_Y_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_Y_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_Y_BIT); }
    else { return -1; }

    writeRegister(BMI160_RA_FOC_CONF, foc_conf);
    writeRegister(BMI160_RA_CMD, BMI160_CMD_START_FOC);
    while (!(readRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) {
    	callbacks.delayMs(1);
    }

    return 0;
}

int BMI160::autoCalibrateZAccelOffset(int target) {
    uint8_t foc_conf;
    if (target == 1) { foc_conf = (0x1 << BMI160_FOC_ACC_Z_BIT); }
    else if (target == -1) { foc_conf = (0x2 << BMI160_FOC_ACC_Z_BIT); }
    else if (target == 0) { foc_conf = (0x3 << BMI160_FOC_ACC_Z_BIT); }
    else { return -1; }

    writeRegister(BMI160_RA_FOC_CONF, foc_conf);
    writeRegister(BMI160_RA_CMD, BMI160_CMD_START_FOC);
    while (!(readRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) {
    	callbacks.delayMs(1);
    }

    return 0;
}

int8_t BMI160::getXAccelOffset(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_0, 1, &readData);
    return readData;
}

int BMI160::setXAccelOffset(int8_t offset) {
	return writeRegister(BMI160_RA_OFFSET_0, offset);
}

int8_t BMI160::getYAccelOffset() {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_1, 1, &readData);
    return readData;
}

int BMI160::setYAccelOffset(int8_t offset) {
	return writeRegister(BMI160_RA_OFFSET_1, offset);
}

int8_t BMI160::getZAccelOffset() {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_2, 1, &readData);
    return readData;
}

int BMI160::setZAccelOffset(int8_t offset) {
	return writeRegister(BMI160_RA_OFFSET_2, offset);
}

uint8_t BMI160::getGyroOffsetEnabled(void) {
    return !!(readRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_EN, 1));
}

int BMI160::setGyroOffsetEnabled(uint8_t enabled) {
	return writeRegisterBits(BMI160_RA_OFFSET_6, enabled ? 0x1 : 0, BMI160_GYR_OFFSET_EN, 1);
}

int BMI160::autoCalibrateGyroOffset(void) {
    uint8_t foc_conf = (1 << BMI160_FOC_GYR_EN);

    writeRegister(BMI160_RA_FOC_CONF, foc_conf);
    writeRegister(BMI160_RA_CMD, BMI160_CMD_START_FOC);
    while (!(readRegBits(BMI160_RA_STATUS, BMI160_STATUS_FOC_RDY, 1))) {
    	callbacks.delayMs(1);
    }

    return 0;
}

int16_t BMI160::getXGyroOffset(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_3, 1, &readData);

	int16_t offset = readData;
	offset |= (int16_t)(readRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_X_MSB_BIT, BMI160_GYR_OFFSET_X_MSB_LEN)) << 8;
    return (int16_t)BMI160_SIGN_EXTEND(offset, 10);
}

int BMI160::setXGyroOffset(int16_t offset) {
	int opStatus = writeRegister(BMI160_RA_OFFSET_3, offset);
	if(opStatus != 0) { return opStatus; }

	opStatus = writeRegisterBits(BMI160_RA_OFFSET_6, offset >> 8, BMI160_GYR_OFFSET_X_MSB_BIT, BMI160_GYR_OFFSET_X_MSB_LEN);
	if(opStatus != 0) { return opStatus; }

	return 0;
}

int BMI160::setYGyroOffset(int16_t offset) {
	int opStatus = writeRegister(BMI160_RA_OFFSET_4, offset);
	if(opStatus != 0) { return opStatus; }

	opStatus = writeRegisterBits(BMI160_RA_OFFSET_6, offset >> 8, BMI160_GYR_OFFSET_Y_MSB_BIT, BMI160_GYR_OFFSET_Y_MSB_LEN);
	if(opStatus != 0) { return opStatus; }

	return 0;
}

int16_t BMI160::getYGyroOffset() {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_4, 1, &readData);

	int16_t offset = readData;
    offset |= (int16_t)(readRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_Y_MSB_BIT, BMI160_GYR_OFFSET_Y_MSB_LEN)) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

int16_t BMI160::getZGyroOffset() {
	uint8_t readData = 0;
	readRegister(BMI160_RA_OFFSET_5, 1, &readData);

	int16_t offset = readData;
    offset |= (int16_t)(readRegBits(BMI160_RA_OFFSET_6, BMI160_GYR_OFFSET_Z_MSB_BIT, BMI160_GYR_OFFSET_Z_MSB_LEN)) << 8;
    return BMI160_SIGN_EXTEND(offset, 10);
}

int BMI160::setZGyroOffset(int16_t offset) {
	int opStatus = writeRegister(BMI160_RA_OFFSET_5, offset);
	if(opStatus != 0) { return opStatus; }

	opStatus = writeRegisterBits(BMI160_RA_OFFSET_6, offset >> 8, BMI160_GYR_OFFSET_Z_MSB_BIT, BMI160_GYR_OFFSET_Z_MSB_LEN);
	if(opStatus != 0) { return opStatus; }

	return 0;
}

uint8_t BMI160::getFreefallDetectionThreshold() {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_LOWHIGH_1, 1, &readData);
    return readData;
}

int BMI160::setFreefallDetectionThreshold(uint8_t threshold) {
	return writeRegister(BMI160_RA_INT_LOWHIGH_1, threshold);
}

uint8_t BMI160::getFreefallDetectionDuration(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_LOWHIGH_0, 1, &readData);
    return readData;
}

int BMI160::setFreefallDetectionDuration(uint8_t duration) {
	return writeRegister(BMI160_RA_INT_LOWHIGH_0, duration);
}

uint8_t BMI160::getShockDetectionThreshold(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_LOWHIGH_4, 1, &readData);
    return readData;
}

int BMI160::setShockDetectionThreshold(uint8_t threshold) {
	return writeRegister(BMI160_RA_INT_LOWHIGH_4, threshold);
}

uint8_t BMI160::getShockDetectionDuration(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_LOWHIGH_3, 1, &readData);
    return readData;
}

int BMI160::setShockDetectionDuration(uint8_t duration) {
	return writeRegister(BMI160_RA_INT_LOWHIGH_3, duration);
}

uint8_t BMI160::getStepDetectionMode(void) {
    uint8_t ret_step_conf0 = 0;
    uint8_t ret_min_step_buf = 0;

	readRegister(BMI160_RA_STEP_CONF_0, 1, &ret_step_conf0);
	readRegister(BMI160_RA_STEP_CONF_1, 1, &ret_min_step_buf);

    if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_NOR) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_NOR)) {
    	return BMI160_STEP_MODE_NORMAL;
    } else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_SEN) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_SEN)) {
    	return BMI160_STEP_MODE_SENSITIVE;
	} else if ((ret_step_conf0 == BMI160_RA_STEP_CONF_0_ROB) && (ret_min_step_buf == BMI160_RA_STEP_CONF_1_ROB)) {
    	return BMI160_STEP_MODE_ROBUST;
    } else {
    	return BMI160_STEP_MODE_UNKNOWN;
    }
}

int BMI160::setStepDetectionMode(BMI160StepMode mode) {
    uint8_t step_conf0 = 0;
    uint8_t min_step_buf = 0;

    switch (mode)
    {
		case BMI160_STEP_MODE_NORMAL:
			step_conf0 = 0x15;
			min_step_buf = 0x3;
			break;
		case BMI160_STEP_MODE_SENSITIVE:
			step_conf0 = 0x2D;
			min_step_buf = 0x0;
			break;
		case BMI160_STEP_MODE_ROBUST:
			step_conf0 = 0x1D;
			min_step_buf = 0x7;
			break;
		default:
			return HAL_ERROR;
    };

    writeRegister(BMI160_RA_STEP_CONF_0, step_conf0);
    writeRegisterBits(BMI160_RA_STEP_CONF_1, min_step_buf, BMI160_STEP_BUF_MIN_BIT, BMI160_STEP_BUF_MIN_LEN);

    return 0;
}

uint8_t BMI160::getStepCountEnabled(void) {
    return !!(readRegBits(BMI160_RA_STEP_CONF_1, BMI160_STEP_CNT_EN_BIT, 1));
}

int BMI160::setStepCountEnabled(uint8_t enabled) {
    return writeRegisterBits(BMI160_RA_STEP_CONF_1, enabled ? 0x1 : 0, BMI160_STEP_CNT_EN_BIT, 1);
}

uint16_t BMI160::getStepCount() {
    uint8_t buffer[2]{};
    //TODO get from device
    return (((uint16_t)buffer[1]) << 8) | buffer[0];
}

int BMI160::resetStepCount(void) {
	return writeRegister(BMI160_RA_CMD, BMI160_CMD_STEP_CNT_CLR);
}

uint8_t BMI160::getMotionDetectionThreshold(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_MOTION_1, 1, &readData);
    return readData;
}

int BMI160::setMotionDetectionThreshold(uint8_t threshold) {
    return writeRegister(BMI160_RA_INT_MOTION_1, threshold);
}

uint8_t BMI160::getMotionDetectionDuration(void) {
    return (1 + readRegBits(BMI160_RA_INT_MOTION_0, BMI160_ANYMOTION_DUR_BIT, BMI160_ANYMOTION_DUR_LEN));
}

int BMI160::setMotionDetectionDuration(uint8_t samples) {
	return writeRegisterBits(BMI160_RA_INT_MOTION_0, samples - 1, BMI160_ANYMOTION_DUR_BIT, BMI160_ANYMOTION_DUR_LEN);
}

uint8_t BMI160::getZeroMotionDetectionThreshold(void) {
	uint8_t readData = 0;
	readRegister(BMI160_RA_INT_MOTION_2, 1, &readData);
    return readData;
}

int BMI160::setZeroMotionDetectionThreshold(const uint8_t threshold) {
    return writeRegister(BMI160_RA_INT_MOTION_2, threshold);
}

uint8_t BMI160::getZeroMotionDetectionDuration(void) {
    return readRegBits(BMI160_RA_INT_MOTION_0, BMI160_NOMOTION_DUR_BIT, BMI160_NOMOTION_DUR_LEN);
}

int BMI160::setZeroMotionDetectionDuration(const uint8_t duration) {
	return writeRegisterBits(BMI160_RA_INT_MOTION_0, duration, BMI160_NOMOTION_DUR_BIT, BMI160_NOMOTION_DUR_LEN);
}

uint8_t BMI160::getTapDetectionThreshold(void) {
    return readRegBits(BMI160_RA_INT_TAP_1, BMI160_TAP_THRESH_BIT, BMI160_TAP_THRESH_LEN);
}

int BMI160::setTapDetectionThreshold(const uint8_t threshold) {
	return writeRegisterBits(BMI160_RA_INT_TAP_1, threshold, BMI160_TAP_THRESH_BIT, BMI160_TAP_THRESH_LEN);
}

uint8_t BMI160::getTapShockDuration(void) {
    return !!(readRegBits(BMI160_RA_INT_TAP_0, BMI160_TAP_SHOCK_BIT, 1));
}

int BMI160::writeRegisterBits(uint8_t reg, uint8_t data, unsigned pos, unsigned len) {
	uint8_t readData = 0;
	int opStatus = readRegister(reg, 1, &readData);

	if(opStatus != 0) {
		return opStatus;
	}

    uint8_t mask = ((1 << len) - 1) << pos;

    data <<= pos;
    data &= mask;
    readData &= ~(mask);
    readData |= data;

    return writeRegister(reg, readData);
}

uint8_t BMI160::readRegBits(uint8_t reg, unsigned pos, unsigned len) {
	uint8_t readData = 0;
	int opStatus = readRegister(reg, 1, &readData);

	if(opStatus != 0) {
		return 0x00;
	}

    uint8_t mask = (1 << len) - 1;
    readData >>= pos;
    readData &= mask;

    return readData;
}

int BMI160::writeRegister(uint8_t address, uint8_t cmd) {
	uint8_t data[2] = {0x00};
	data[0] = address;
	data[1] = cmd;

	return callbacks.transmit(this->i2cAddress, data, 2);
}

int BMI160::readRegister(uint8_t address, uint8_t dataSize, uint8_t *rec) {
	uint8_t data[1] = {0x00};
	data[0] = address;

	callbacks.transmit(this->i2cAddress, data, 1);
	return callbacks.receive(this->i2cAddress, &rec[0], dataSize);
}

