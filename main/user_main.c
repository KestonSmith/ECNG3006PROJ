/*
	Name: Keston Smith
	ID: 816001854
	Copyright: 
	Author: Keston Smith
	Date: 18/12/20 02:44
	Description: LAB 2
*/

/* I2C example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/adc.h"

static const char *TAG = "main";

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a ADS1115 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO2 is assigned as the data signal of i2c master port
 *    GPIO0 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO2/GPIO0
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define I2C_EXAMPLE_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           0               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

     
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x01              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x00              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x00              /*!< I2C ack value */
#define NACK_VAL                            0x01              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x02              /*!< I2C last_nack value */

/**
 * Define the ADS1115 register address:
 */
#define ADS1115_addr						0x48
#define CONVERSION_REG						0x00
#define CONFIG_REG							0x01
#define LO_THRESH							0x02
#define HI_THRESH							0x03
//#define CONFIG_REG_MODE						0x

uint16_t sum=0;
uint16_t adc_AN0[20];
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write ADS1115
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */




/**
 * @brief test code to read ADS1115
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_example_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


static esp_err_t i2c_example_master_ads1115_init(i2c_port_t i2c_num){
    
    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
   i2c_example_master_init();
    cmd_data = 0x01;    // Set the FSR to 4.096V
    ESP_ERROR_CHECK(i2c_example_master_ads1115_write(i2c_num, CONFIG_REG, &cmd_data, 1));
   
    return ESP_OK;
}
   //*********Keston Smith-816001854**************************************//
int average(int sum, int amount){
	
	return sum/amount;
	
}
static void i2c_task_example(void *arg){
  
    i2c_example_master_init(I2C_EXAMPLE_MASTER_NUM);
 
 // adc_data [0]/60 = HI_THRESH;
 //adc_data [0]%60 = LO_THRESH
 int count =0;
  int x=0;
    

        if (ESP_OK == adc_read_fast(adc_AN0, 20)) {
        	 ESP_LOGI(TAG, "Analysing your Heart Rate. Please Wait/n");
            for (x = 0; x < 19; x++) {
            	count= sum*6;
                sum=sum+adc_AN0[x];
                 vTaskDelay(500 / portTICK_RATE_MS);
			    //ESP_LOGI(TAG, "Analysing your Heart Rate. Please Wait/n");
              
            }
            int avg = average(count, 20);
            
            int heart=(avg*2)/15;
            
           ESP_LOGI(TAG, "Your Heart Rate is: %d beats/min/n", heart);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);

}

void app_main(void)
{
	 adc_config_t adc_config;
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8; // ADC sample collection clock = 80MHz/clk_di(v = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));
    //start i2c task
    //start i2c task
    xTaskCreate(i2c_task_example, "i2c_task_example", 2048, NULL, 10, NULL);
    
    for(;;);
}
