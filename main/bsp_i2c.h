#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif
/*C语言头文件包含*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"

/*如果有其他C语言的类型、变量或函数声明，也可以放在这里*/
//...
#define I2C_MASTER_SCL_IO           10                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C0 port number    */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000                       /*!< I2C master timeout can't overtop 1s*/

// typedef enum {
//     I2C_MASTER_WRITE = 0,   /*!< I2C write data */
//     I2C_MASTER_READ,        /*!< I2C read data */
// } i2c_rw_t;

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write(写:0) */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read(读:1) */
#define ACK_CHECK_EN                0x1         /*!< I2C master will check ack from slave(主机检查从机的ACK)*/
#define ACK_CHECK_DIS               0x0         /*!< I2C master will not check ack from slave(主机不检查从机的ACK) */
#define ACK_VAL                     0x0         /*!< I2C ack value(应答) */
#define NACK_VAL                    0x1         /*!< I2C nack value(不应答) */



// #define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */

// /*Function declaration*/
esp_err_t i2c_master_read_slave (i2c_port_t i2c_num, uint8_t ESP_SLAVE_ADDR, uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t ESP_SLAVE_ADDR, uint8_t *data_wr, size_t size);
// static esp_err_t Device_i2c_init(void);
esp_err_t Sensor_i2c_master_init(void);

#ifdef __cplusplus
}
#endif

//如果这个头文件同时被C++代码使用，那么可以在这里添加C++特有的代码或声明
//...

#endif