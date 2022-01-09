/**
 * @brief       test_pwm_freq.cpp
 * @details     Test setting of frequency and duty cycle
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


TEST_CASE("Set duty cycle, LED0", "[pca9685]")
{
  PCA9685 *pca9685 = new PCA9685(I2C_ADDR, i2c_sda, i2c_scl);
  pca9685->set_duty_cycle(0, 0, PCA9685::MAX_PWM_COUNT/2);
  pca9685->~PCA9685();
}

TEST_CASE("Set refresh frequency", "[pca9685]")
{
  PCA9685 *pca9685 = new PCA9685(I2C_ADDR, i2c_sda, i2c_scl);
  pca9685->set_refresh_freq(500);
  pca9685->~PCA9685();
}
