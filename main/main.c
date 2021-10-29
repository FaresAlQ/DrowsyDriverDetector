#include <stdio.h>
#include "sensors.c"
#include "SparkFun_Bio_Sensor_Hub_Library.cpp"

static const char *TAG_M = "main.c";

void app_main(void)
{
	ESP_LOGI(TAG_M, "~~~PROGRAM STARTS HERE~~~");
	init_sensors();
}
