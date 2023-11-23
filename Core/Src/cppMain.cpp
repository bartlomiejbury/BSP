#include <BMI160.h>
#include <Magneto/DFRobot_BMM150.h>
#include "os.h"
#include "trace.h"
#include "usbd_cdc_if.h"
#include "ILI9341Lcd.h"
#include "ili9341/ili9341.h"
#include "colors.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart6;

uint8_t buffer[128];

typedef struct KalmanOutput {
	float Value;
	float Uncertainty;
} KalmanOutput_t;


KalmanOutput_t kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {

	KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

    KalmanOutput_t result = {};
    result.Value = KalmanState;
    result.Uncertainty = KalmanUncertainty;

    return result;
}


int BMI160_Delay(uint32_t timeout) {
	HAL_Delay(timeout);
	return 0;
}

int BMI160_Transmit(uint16_t address,  uint8_t *data, uint16_t size) {
	HAL_StatusTypeDef opResoult = HAL_I2C_Master_Transmit(&hi2c1, address, data, size, 10);
	return opResoult == HAL_OK ? 0 : -1;
}

int BMI160_Receive(uint16_t address,  uint8_t *data, uint16_t size) {
	HAL_StatusTypeDef opResoult = HAL_I2C_Master_Receive(&hi2c1, address, data, size, 10);
	return opResoult == HAL_OK ? 0 : -1;
}

KalmanOutput_t KalmanRoll = {0, 2*2};
KalmanOutput_t KalmanPitch = {0, 2*2};
BMI160 *bmi160 = NULL;

extern "C" void DefaultTask() {
	{
		I2C_Callbacks clbks = {
			.transmit = BMI160_Transmit,
			.receive = BMI160_Receive,
			.delayMs = BMI160_Delay
		};

		BMI160_Config config {
			.accelRange = BMI160_ACCEL_RANGE_4G,
			.accelRate = BMI160_ACCEL_RATE_100HZ,
			.accelmode = BMI160_DLPF_MODE_OSR2,
			.gyroRange = BMI160_GYRO_RANGE_250,
			.gyroRate = BMI160_GYRO_RATE_100HZ,
			.gyroMode = BMI160_DLPF_MODE_OSR2
		};

		bmi160 = new BMI160(BMI160_I2C_ADDR << 1, &clbks);
		bmi160->init(&config);
	}

    while(1) {

    	GyroAccelData gyro = bmi160->scaledGyroData();
    	AnglePeachData angles = bmi160->getAccelerometerAngles();
    	KalmanRoll = kalman_1d(KalmanRoll.Value, KalmanRoll.Uncertainty, gyro.x, angles.AngleRoll);
    	KalmanPitch = kalman_1d(KalmanPitch.Value, KalmanPitch.Uncertainty, gyro.y, angles.AnglePitch);

    	int size = snprintf((char*)buffer, 128, "Roll Angle [o] %d Pitch Angle [o] %d", (int)(KalmanRoll.Value), (int)(KalmanPitch.Value));
    	if (size > 128) {
    		size = 128;
    	}

    	HAL_UART_Transmit(&huart6, buffer, size, 10);
    	HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, 10);
		HAL_Delay(50);
    }

}
