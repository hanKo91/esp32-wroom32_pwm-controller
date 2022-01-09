/**
 * @brief       main.c
 * @details     Main test application
 * @return      NA
 *
 * @author      
 * @date        
 * @version     
 */
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

void app_main(void)
{
  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();
}
