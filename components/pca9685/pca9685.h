/**
 * @brief       PCA9685.h
 * @details     16-channel, 12-bit PWM I2C-bus controller.
 * @return      NA
 *
 * @author
 * @date
 * @version
 */

#pragma once

#include "driver/i2c.h"


class PCA9685
{
  public:
    // Registers addresses
    static const uint8_t MODE1          = 0x00;  /*  Mode register 1 */
    static const uint8_t MODE2          = 0x01;  /*  Mode register 2 */
    static const uint8_t SUBADR1        = 0x02;  /*  I2C-bus subaddress 1 */
    static const uint8_t SUBADR2        = 0x03;  /*  I2C-bus subaddress 2 */
    static const uint8_t SUBADR3        = 0x04;  /*  I2C-bus subaddress 3 */
    static const uint8_t ALLCALLADR     = 0x05;  /*  LED All Call I2C-bus address */
    static const uint8_t LED0_ON_L      = 0x06;  /*  LED0 output and brightness control byte 0 */
    static const uint8_t LED0_ON_H      = 0x07;  /*  LED0 output and brightness control byte 1 */
    static const uint8_t LED0_OFF_L     = 0x08;  /*  LED0 output and brightness control byte 2 */
    static const uint8_t LED0_OFF_H     = 0x09;  /*  LED0 output and brightness control byte 3 */
    static const uint8_t ALL_LED_ON_L   = 0xFA;  /*  ALL_LED output and brightness control byte 0 */
    static const uint8_t ALL_LED_ON_H   = 0xFB;  /*  ALL_LED output and brightness control byte 1 */
    static const uint8_t ALL_LED_OFF_L  = 0xFC;  /*  ALL_LED output and brightness control byte 2 */
    static const uint8_t ALL_LED_OFF_H  = 0xFD;  /*  ALL_LED output and brightness control byte 3 */
    static const uint8_t PRE_SCALE      = 0xFE;  /*  prescaler for PWM output frequency */
    static const uint8_t TESTMODE       = 0xFF;  /*  defines the test mode to be entered */
    // MODE1 bits
    static const uint8_t MODE1_ALLCAL   = 0;  /* respond to LED All Call I2C-bus address */
    static const uint8_t MODE1_SUB3     = 1;  /* respond to I2C-bus subaddress 3 */
    static const uint8_t MODE1_SUB2     = 2;  /* respond to I2C-bus subaddress 2 */
    static const uint8_t MODE1_SUB1     = 3;  /* respond to I2C-bus subaddress 1 */
    static const uint8_t MODE1_SLEEP    = 4;  /* Low power mode. Oscillator off */
    static const uint8_t MODE1_AI       = 5;  /* Auto-Increment enabled */
    static const uint8_t MODE1_EXTCLK   = 6;  /* Use EXTCLK pin clock */
    static const uint8_t MODE1_RESTART  = 7;  /* Restart enabled */
    // MODE2 bits
    static const uint8_t MODE2_OUTNE_0  = 0;  /* Active LOW output enable input */
    static const uint8_t MODE2_OUTNE_1  = 1;  /* Active LOW output enable input - high impedience */
    static const uint8_t MODE2_OUTDRV   = 2;  /* totem pole structure vs open-drain */
    static const uint8_t MODE2_OCH      = 3;  /* Outputs change on ACK vs STOP */
    static const uint8_t MODE2_INVRT    = 4;  /* Output logic state inverted */
    // Defaults & Constants
    static const uint16_t MAX_REFRESH_RATE  = 1526;     /* Maximum refresh rate, Hz */
    static const uint16_t MIN_REFRESH_RATE  = 24;       /* Minimum refresh rate, Hz */
    static const uint16_t MAX_PWM_COUNT     = 4095;     /* Maximum value of 12 bit counter */
    static const uint8_t LED_STEP_FACTOR    = 4;        /* Factor to calculate next LED */
    static const uint32_t CLK_FREQ          = 25000000; /* PCA9685 internal clock frequency */
    static const uint32_t REFRESH_FREQ      = 200;      /* Default PWM frequency */
    // I2C Settings
    static const uint8_t I2C_NUM          = I2C_NUM_0; /* Default I2C bus */
    static const uint8_t I2C_ADDR         = 0x70;      /* Default PCA9685 I2C Slave Address */
    static const uint32_t I2C_CLK_SPEED   = 400000;    /* Default master clock speed */
    static const uint32_t I2C_TIMEOUT_MS  = 1000;      /* Default timeout_ms */
    // I2C User Settings
    uint32_t i2c_num;         /* User-specified I2C bus */
    uint32_t i2c_addr;        /* User-specified PCA9685 I2C Slave Address */
    uint32_t i2c_clk_speed;   /* User-specified master clock speed */
    uint32_t i2c_timeout_ms;  /* User-specified timeout_ms */

    PCA9685(uint32_t i2c_num, uint32_t i2c_addr, gpio_num_t i2c_sda, gpio_num_t i2c_scl, uint32_t i2c_clk_speed, uint32_t i2c_timeout_ms);
    PCA9685(uint32_t i2c_addr, gpio_num_t i2c_sda, gpio_num_t i2c_scl);
    ~PCA9685();
    void write8(uint8_t i2c_reg_addr, uint8_t data);
    uint8_t read8(uint8_t i2c_reg_addr);
    void set_duty_cycle(uint8_t led, uint16_t on, uint16_t off);
    void set_refresh_freq(uint16_t frequency);
    void wakeup();
    void sleep();
    void restart();

  private:
    uint8_t toggle_bit(uint8_t num, uint8_t bit_nr);
    uint8_t set_bit(uint8_t num, uint8_t bit_nr);
    uint8_t clear_bit(uint8_t num, uint8_t bit_nr);
    uint8_t check_bit(uint8_t num, uint8_t bit_nr);
};
