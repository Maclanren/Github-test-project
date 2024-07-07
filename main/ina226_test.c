#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "INA226.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_i2c.h"

extern const char *TAG_Sensor;
extern const char *TAG_INA226;


void app_main(void)
{
    float Busvoltage = 0.0,Current = 0.0;

    ESP_ERROR_CHECK(Sensor_i2c_master_init());
    ESP_LOGI(TAG_Sensor, "I2C initialized successfully");
    vTaskDelay(500 / portTICK_RATE_MS);
    while(1)
    {
        INA226_Get_Data(&Busvoltage,&Current);
        ESP_LOGI(TAG_INA226, "Bus power: %f", Busvoltage);
        ESP_LOGI(TAG_INA226, "Shunt current: %f", Current);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));   //删除I2C驱动程序
    ESP_LOGI(TAG_INA226, "I2C unitialized successfully"); //打印I2C释放成功log
}
