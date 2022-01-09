/**
 * @brief       test_rw.c
 * @details     Test read/write
 * @return      NA
 *
 * @author      
 * @date        
 * @version     
 */
#include <limits.h>
#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "pca9685.h"

const gpio_num_t i2c_sda = GPIO_NUM_21;
const gpio_num_t i2c_scl = GPIO_NUM_22;
#define I2C_ADDR 0x70
#define I2C_REG_ADDR 0x06

TEST_CASE("Write/Read register with main api", "[pca9685]")
{
  // initialization
  esp_err_t ret;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = i2c_sda;
  conf.scl_io_num = i2c_scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
  // Wait for initialization to finish
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // read/write 
  uint8_t data = 0x17;
  uint8_t write_buf[2] = {I2C_REG_ADDR, data};
  uint8_t read_buf = 0x00;
  ret = i2c_master_write_to_device(I2C_NUM_0, I2C_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_RATE_MS);
  TEST_ASSERT(ret == ESP_OK);
  ret = i2c_master_read_from_device(I2C_NUM_0, I2C_ADDR, &read_buf, sizeof(read_buf), 1000 / portTICK_RATE_MS);
  TEST_ASSERT(ret == ESP_OK);
  TEST_ASSERT(data == read_buf);
  ret = i2c_driver_delete(I2C_NUM_0);
  TEST_ASSERT(ret == ESP_OK);
}

TEST_CASE("Write/Read register with class wrapper - single byte", "[pca9685]")
{
  uint8_t data_w = 0x17;
  PCA9685 *pca9685 = new PCA9685(I2C_ADDR, i2c_sda, i2c_scl);

  pca9685->write8(I2C_REG_ADDR, data_w);
  uint8_t data_r = pca9685->read8(I2C_REG_ADDR);

  TEST_ASSERT(data_w == data_r);
  pca9685->~PCA9685();
}
