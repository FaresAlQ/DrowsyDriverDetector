#include <stdio.h>
//#include <Wire.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
//#include "SparkFun_Bio_Sensor_Hub_Library.h"
//#include "SparkFun_Bio_Sensor_Hub_Library.cpp"
#include <time.h>

void triggerSpeaker();

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

// Speaker pin
#define GPIO_OUTPUT 21

int resPin = 4;
int mfioPin = 5;

static xQueueHandle gpio_evt_queue = NULL;

//SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);

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
	return num / 25 * 0.1;
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

int hRcounter = 1;
float hRsum = 0;

int heartRateThresh(float hRdata){
   float avg = 0;
   hRsum += hRdata; // new data point added to sum
   if(hRcounter < 20){ avg = hRsum / hRcounter;hRcounter++;}// if we have less than 20 data points in average: Compute average //////WORKS
  else{hRcounter++;}

   // otherwise compare each data point to average
    if((hRdata < (avg*0.9)) && hRcounter >= 20){ // 10% decrease
    printf("hR average is: %f\n",avg);
      printf("HR ALARMING VALUE\n");
      printf("Datapoint corresponding is: ");
      printf("%f\n",hRdata);

      printf("This ocurred on the %dth try\n",hRcounter);
      return 1;
   }

   return 0;

}

int gyroCt = 1;
float gyroSum = 0;

int gyroThresh(float angle){
   // WILL RETURN VALUE OF 1 FOR ALARMING VALUE

   if(angle >=90){return 1;} // for edge case
   float avg = 0;
   gyroSum += angle;
   if(gyroCt < 20){ avg = gyroSum / gyroCt;gyroCt++;} // if we have less than 20 data points in average: Compute average //////WORKS
   else{
      // printf("Angle Average is:");
      // printf("%f \n",avg);
      gyroCt++;
   }
   // otherwise compare each data point to average
    if((angle > 10)){//(avg*1.2)) && (gyroCt >= 20)){ // 10% decrease
      printf("ANGLE ALARMING VALUE\n");
      printf("angle corresponding is: ");
      printf("%f\n",angle);
      printf("This ocurred on the %dth try\n",gyroCt);
      printf("Angle average is: ");
      printf("%f\n",avg);
      triggerSpeaker();
      return 1;
   }

   return 0;
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void triggerSpeaker(){

	//zero-initialize the config structure.
	gpio_config_t io_conf = {};
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = (1ULL<<GPIO_OUTPUT);
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	//change gpio interrupt type for one pin
	gpio_set_intr_type(GPIO_OUTPUT, GPIO_INTR_ANYEDGE);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	gpio_set_level(GPIO_OUTPUT, 1);
	time_t begin,end;
	begin= time(NULL);

	while(1){ // wait to begin calculating
	      	end = time(NULL);
			//printf("waiting");
			if((difftime(end,begin)) > 3){
				break;
	}}
	gpio_set_level(GPIO_OUTPUT, 0);
}

void init_sensors(){
	uint8_t gyro_data[2];
	uint8_t heart_data[2];
	float gyroX;
	float gyroY;
	float gyroZ;

	float heart_rate;
	ESP_ERROR_CHECK(i2c_master_init());
	ESP_LOGI(TAG_S, "I2C initialized successfully");

	/* Demonstrate writing by reseting the MPU9250 */
	ESP_ERROR_CHECK(mpu_register_write_byte(MPU_PWR_MGMT_1_REG_ADDR, 1 << GYRO_RESET_BIT));

	//ESP_ERROR_CHECK(max_register_read(HEART_RATE_ADDR, heart_data, 2));
	//ESP_LOGI(TAG_S, "heart rate = %X %X", heart_data[0], heart_data[1]);
	while(1){

	time_t begin,end;
	begin= time(NULL);

	   while(1){ // wait to begin calculating
      	end = time(NULL);
		//printf("waiting");
		if((difftime(end,begin)) > 0.1){
			break;
		}
		}

		/* Read the MPU GYRO_XOUT register */
		ESP_ERROR_CHECK(mpu_register_read(GYRO_X_ADDR, gyro_data, 2));
		gyroX = get_angle(byte_to_int(gyro_data[0], gyro_data[1])); // int gyro_x
		ESP_LOGI(TAG_S, "angle in X direction = %f\n", gyroX);



		/* Read the MPU GYRO_YOUT register
		ESP_ERROR_CHECK(mpu_register_read(GYRO_Y_ADDR, gyro_data, 2)); // int gyro_y
		gyroY = get_angle(byte_to_int(gyro_data[0], gyro_data[1]));
		ESP_LOGI(TAG_S, "angle in Y direction = %f", gyroY);

		 Read the MPU GYRO_XOUT register
		ESP_ERROR_CHECK(mpu_register_read(GYRO_Z_ADDR, gyro_data, 2)); // int gyro_z
		gyroZ = get_angle(byte_to_int(gyro_data[0], gyro_data[1]));
		ESP_LOGI(TAG_S, "angle in Z direction = %f\n", gyroZ);*/

		int value = gyroThresh(gyroX); // 1 for alarm 0 for nothing
		/*if(value == 1){
			printf("ALARM\n");
			break;}
*/



	}
	/* Demonstrate writing by reseting the MPU9250 */
	ESP_ERROR_CHECK(mpu_register_write_byte(MPU_PWR_MGMT_1_REG_ADDR, 1 << GYRO_RESET_BIT));

	ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
	ESP_LOGI(TAG_S, "I2C unitialized successfully");
}
