/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
 xQueueHandle gpio_evt_queue = NULL;
TaskHandle_t xClear_ISR_Handle;
volatile uint16_t count_ISR = 0;
#define TAG_SENSOR "SENSOR"
/*************I2C DEFINE*****************/
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define CONFIG_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SDA 21
#define CONFIG_I2C_MASTER_PORT_NUM 1
#define CONFIG_I2C_MASTER_FREQUENCY 100000
#define CONFIG_I2C_SLAVE_SCL 5
#define CONFIG_I2C_SLAVE_SDA 4
#define CONFIG_I2C_SLAVE_PORT_NUM 0
#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/*************GPIO DEFINE*****************/
#define GPIO_INPUT_IO_0     0
//#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0

#define MC3416
/*************MC3416 REGISTER***************/
#ifdef MC3416
#define MC3413_I2C_ADDR         0x4C
#define DEV_STAT                0x05
#define INTR_CTRL               0x06
#define MODE                    0x07
#define SR                      0x08
#define MOTION_CTRL             0x09
#define XOUT_EX_L               0x0D
#define XOUT_EX_H               0x0E
#define YOUT_EX_L               0x0F
#define YOUT_EX_H               0x10
#define ZOUT_EX_L               0x11
#define ZOUT_EX_H               0x12
#define STATUS_2                0x13
#define INTR_STAT_2             0x14
#define RANGE                   0x20
#define XOFFL                   0x21
#define XOFFH                   0x22
#define YOFFL                   0x23
#define YOFFH                   0x24
#define ZOFFL                   0x25
#define ZOFFH                   0x26
#define XGAIN                   0x27
#define YGAIN                   0x28
#define ZGAIN                   0x29
#define TF_THRESH_LSB           0x40
#define TF_THRESH_MSB           0x41
#define TF_DB                   0x42
#define AM_THRESH_LSB           0x43
#define AM_THRESH_MSB           0x44
#define AM_DB                   0x45
#define SHK_THRESH_LSB          0x46
#define SHK_THRESH_MSB          0x47
#define PK_P2P_DUR_THRESH_LSB   0x48
#define PK_P2P_DUR_THRESH_MSB   0x49
#define TIMER_CTRL              0x4A
#endif

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static esp_err_t i2c_WriteByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err write: %d\n", ret);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1	 / portTICK_PERIOD_MS);
}
static esp_err_t i2c_ReadByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t* Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	//i2c_master_stop(cmd);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, Data, NACK_VAL);
	i2c_master_stop(cmd);

	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err read: %d\n", ret);
	//printf("read: %02x\n", *Data);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1 / portTICK_PERIOD_MS);
}
void clear_interrupt_source(void)
{
	uint8_t read = 0;
#ifdef MC3416
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_STAT_2, &read);
		//printf("ISR: %02x\r\n", read);
		//vTaskDelay(1 / portTICK_RATE_MS);
	}
#endif
#ifdef MC3413
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
		//printf("ISR: %02x\r\n", read);
		//vTaskDelay(1 / portTICK_RATE_MS);
	}
#endif
}
void acc_power_down(void)
{
	uint8_t read;
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x10);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power down\r\n");
}
 void acc_power_up(void)
{
    uint8_t read;
    i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x11);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power up\r\n");
}
static void clear_ISR_task(void *arg)
{
	uint32_t io_num;
	    for(;;)
	    {
			if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
			{
				count_ISR++;
				clear_interrupt_source();
				ESP_LOGW(TAG_SENSOR, "%d \r\n", count_ISR++);
				vTaskDelay(50/portTICK_RATE_MS);
			}
	    }
}

static void IRAM_ATTR gpio_sensor_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
void gpio_init(void)
{
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	io_conf.pin_bit_mask =  ((1ULL<<2));
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	xTaskCreate(clear_ISR_task, "clear_ISR_task", 1024*2, NULL, 5, NULL);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(2, gpio_sensor_isr_handler, (void*) 2);
	clear_interrupt_source();
}
void acc_config(void)
{
	uint8_t read = 0;

#ifdef MC3416
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x10); // 0x00
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, 0x0f); // 0x1f
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, &read);
	ESP_LOGI(TAG_SENSOR,"INTR_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, 0x3f); //0x3f
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, &read);
	ESP_LOGI(TAG_SENSOR, "MOTION_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, 0x09); //0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, &read);
	ESP_LOGI(TAG_SENSOR,"RANGE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, 0x04); // 0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
	ESP_LOGI(TAG_SENSOR,"SR: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"XGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"YGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, 0x0f);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"ZGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, 0x02);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x11);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);
	clear_interrupt_source();
#endif
}
void check_motion(void * arg)
{
	while(1)
	{
		uint8_t read = 0;
		uint16_t read_f = 0;
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XOUT_EX_H, &read);
		read_f = read;
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XOUT_EX_L, &read);
		read_f = read_f << 8 | read;
		printf("%x ", read_f);
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YOUT_EX_H, &read);
		read_f = read;
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YOUT_EX_L, &read);
		read_f = read_f << 8 | read;
		printf("%x ", read_f);
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZOUT_EX_H, &read);
		read_f = read;
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZOUT_EX_L, &read);
		read_f = read_f << 8 | read;
		printf("%x\r\n", read_f);

	}
}
void app_main(void)
{
	i2c_master_init();
	acc_config();
	gpio_init();
	xTaskCreate(check_motion, "check_motion", 1024*8, NULL, 5, NULL);
}
