#include <stdio.h>
#include <Wire.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "SparkFun_Bio_Sensor_Hub_Library.h"
#include "SparkFun_Bio_Sensor_Hub_Library.cpp"

static const char *TAG_S = "sensors.c";

#define I2C_MASTER_SCL_IO			22
#define I2C_MASTER_SDA_IO			23
#define I2C_MASTER_NUM				0
#define I2C_MASTER_FREQ_HZ			400000
#define I2C_MASTER_TX_BUF_DISABLE	0
#define I2C_MASTER_RX_BUF_DISABLE	0
#define I2C_MASTER_TIMEOUT_MS		1000

// MPU6050 - gyro sensor
#define MPU_SENSOR_ADDR			0x68
#define GYRO_X_ADDR				0x43
#define GYRO_Y_ADDR 			0x45
#define GYRO_Z_ADDR 			0x47
#define MPU_WHO_AM_I_REG_ADDR	0x75
#define MPU_PWR_MGMT_1_REG_ADDR	0x6B
#define GYRO_RESET_BIT			2

// MAX32664 - heart rate sensor
#define MAX_SENSOR_ADDR			0x13
#define HEART_RATE_ADDR			0x01
/* Unsure of these values
#define MAX_WHO_AM_I_REG_ADDR	0x75
#define MAX_PWR_MGMT_1_REG_ADDR	0x6B
#define MAX_RESET_BIT			7*/

int resPin = 4;
int mfioPin = 5;

SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);

int byte_to_int(uint8_t data_0, uint8_t data_1){
	int ret;
	if (!(data_0 & 0x80)){
		ret = data_0 << 8 | data_1;
	}
	else{
		ret = ((data_0 ^ 0xFF) << 8) | (data_1 ^ 0xFF);
		ret = -1 * (ret + 1);
	}
	return ret;
}

float get_angle(int num){
	return num / 250.0 * 0.1;
}

static esp_err_t mpu_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t mpu_register_write_byte(uint8_t reg_addr, uint8_t data) {
	int ret;
	uint8_t write_buf[2] = {reg_addr, data};
	ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

	return ret;
}

static esp_err_t max_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t max_register_write_byte(uint8_t reg_addr, uint8_t data) {
	int ret;
	uint8_t write_buf[2] = {reg_addr, data};
	ret = i2c_master_write_to_device(I2C_MASTER_NUM, MAX_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

	return ret;
}

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void init_sensors(){
	uint8_t gyro_data[2];
	uint8_t heart_data[2];
	float gyro;
	float heart_rate;
	ESP_ERROR_CHECK(i2c_master_init());
	ESP_LOGI(TAG_S, "I2C initialized successfully");

	ESP_ERROR_CHECK(max_register_read(HEART_RATE_ADDR, heart_data, 2));
	ESP_LOGI(TAG_S, "heart rate = %X %X", heart_data[0], heart_data[1]);
	while(1){
		/* Read the MPU GYRO_XOUT register */
		ESP_ERROR_CHECK(mpu_register_read(GYRO_X_ADDR, gyro_data, 2));
		gyro = get_angle(byte_to_int(gyro_data[0], gyro_data[1]));
		ESP_LOGI(TAG_S, "angle in X direction = %f", gyro);

		/* Read the MPU GYRO_YOUT register */
		ESP_ERROR_CHECK(mpu_register_read(GYRO_Y_ADDR, gyro_data, 2));
		gyro = get_angle(byte_to_int(gyro_data[0], gyro_data[1]));
		ESP_LOGI(TAG_S, "angle in Y direction = %f", gyro);

		/* Read the MPU GYRO_XOUT register */
		ESP_ERROR_CHECK(mpu_register_read(GYRO_Z_ADDR, gyro_data, 2));
		gyro = get_angle(byte_to_int(gyro_data[0], gyro_data[1]));
		ESP_LOGI(TAG_S, "angle in Z direction = %f\n", gyro);
	}
	/* Demonstrate writing by reseting the MPU9250 */
	ESP_ERROR_CHECK(mpu_register_write_byte(MPU_PWR_MGMT_1_REG_ADDR, 1 << GYRO_RESET_BIT));

	ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
	ESP_LOGI(TAG_S, "I2C unitialized successfully");
}