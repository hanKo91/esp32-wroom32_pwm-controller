/**
 * @brief       PCA9685.c
 * @details     16-channel, 12-bit PWM I2C-bus controller.
 * @return      NA
 *
 * @author      
 * @date        
 * @version     
 */
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "pca9685.h"


//************************************************************************************************ PRIVATE

uint8_t PCA9685::toggle_bit(uint8_t num, uint8_t bit_nr)   {return num ^=  (1 << bit_nr);}
uint8_t PCA9685::set_bit(uint8_t num, uint8_t bit_nr)      {return num |=  (1 << bit_nr);}
uint8_t PCA9685::clear_bit(uint8_t num, uint8_t bit_nr)    {return num ^=  (1 << bit_nr);}
uint8_t PCA9685::check_bit(uint8_t num, uint8_t bit_nr)    {return num &   (1 << bit_nr);}


//************************************************************************************************ PUBLIC

PCA9685::PCA9685(uint32_t i2c_num, uint32_t i2c_addr, gpio_num_t i2c_sda, gpio_num_t i2c_scl, uint32_t i2c_clk_speed, uint32_t i2c_timeout_ms)
{
  PCA9685::i2c_num = i2c_num;
  PCA9685::i2c_addr = i2c_addr;
  PCA9685::i2c_clk_speed = i2c_clk_speed;
  PCA9685::i2c_timeout_ms = i2c_timeout_ms;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = i2c_sda;
  conf.scl_io_num = i2c_scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  conf.master.clk_speed = i2c_clk_speed;
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_param_config(i2c_num, &conf));
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

PCA9685::PCA9685(uint32_t i2c_addr, gpio_num_t i2c_sda, gpio_num_t i2c_scl)
{
  PCA9685::i2c_num = I2C_NUM;
  PCA9685::i2c_addr = i2c_addr;
  PCA9685::i2c_clk_speed = I2C_CLK_SPEED;
  PCA9685::i2c_timeout_ms = I2C_TIMEOUT_MS;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = i2c_sda;
  conf.scl_io_num = i2c_scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  conf.master.clk_speed = I2C_CLK_SPEED;
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_param_config(I2C_NUM, &conf));
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0));
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

PCA9685::~PCA9685()
{
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_delete(i2c_num));
}

void PCA9685::write8(uint8_t i2c_reg_addr, uint8_t data)
{
  uint8_t buffer[2] = {i2c_reg_addr, data};
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_to_device(i2c_num, i2c_addr, buffer, 2, i2c_timeout_ms / portTICK_RATE_MS));
}

uint8_t PCA9685::read8(uint8_t i2c_reg_addr)
{
  uint8_t buffer = i2c_reg_addr;
  uint8_t data = 0x00;
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_to_device(i2c_num, i2c_addr, &buffer, 1, i2c_timeout_ms / portTICK_RATE_MS));
  ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read_from_device(i2c_num, i2c_addr, &data, 1, i2c_timeout_ms / portTICK_RATE_MS));
  return data;
}

void PCA9685::set_duty_cycle(uint8_t led, uint16_t on, uint16_t off)
{
  sleep();

  // limits
  if (on  > MAX_PWM_COUNT) on = MAX_PWM_COUNT;
  if (off > MAX_PWM_COUNT) off = MAX_PWM_COUNT;

  // split uint16_t values to H and L bytes
  uint8_t ON_L  = (uint8_t) (on & 0x00FF);
  uint8_t ON_H  = (uint8_t) ((on & 0xFF00) >> 8);
  uint8_t OFF_L = (uint8_t) (off & 0x00FF);
  uint8_t OFF_H = (uint8_t) ((off & 0xFF00) >> 8);

  // calculate offset for LEDx, x > 0
  uint8_t offset = LED_STEP_FACTOR * (uint8_t)led;

  write8(LED0_ON_L + offset, ON_L);
  write8(LED0_ON_H + offset, ON_H);
  write8(LED0_OFF_L + offset, OFF_L);
  write8(LED0_OFF_H + offset, OFF_H);

  // wakeup and restart
  uint8_t mode1_reg = read8(MODE1);
  if(check_bit(mode1_reg, MODE1_RESTART))
  {
    wakeup();
  }
  restart();
}

void PCA9685::set_refresh_freq(uint16_t frequency)
{
  sleep();

  // limits
  if(frequency < MIN_REFRESH_RATE) {
    frequency = MIN_REFRESH_RATE;
  } else if(frequency > MAX_REFRESH_RATE) {
    frequency = MAX_REFRESH_RATE;
  }

  // calculate prescale value
  double pres = round(double(CLK_FREQ) / ((double)frequency * 4096)) - 1;
  uint8_t prescale = (uint8_t)pres;

  write8(PRE_SCALE, prescale);

  // wakeup
  wakeup();
}

void PCA9685::wakeup()
{
  uint8_t state = read8(MODE1);
  uint8_t wake_mode = clear_bit(state, MODE1_SLEEP); 
  write8(MODE1, wake_mode);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void PCA9685::sleep()
{
  uint8_t state = read8(MODE1);
  uint8_t sleep_mode = set_bit(state, MODE1_SLEEP); 
  write8(MODE1, sleep_mode);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void PCA9685::restart()
{
  uint8_t state = read8(MODE1);
  uint8_t restart = set_bit(state, MODE1_RESTART);
  write8(MODE1, restart);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
