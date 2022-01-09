# ESP32 PWM Controller

PWM Controller using ESP32 and PCA9685, cabable of controlling multiple Servos via I2C.

## Hardware

* [ESP32-D1-mini](https://www.az-delivery.de/products/esp32-d1-mini)
* [PCA9685](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)

## Dependencies

* [ESP-IDF](https://github.com/espressif/esp-idf) >= v4.4

## Build

See installation guide for Windows
* [docs.espressif/windows-setup](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html)
* [docs.espressif/windows-setup-update](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup-update.html)

```powershell
# Open Powershell
# ESP_IDF_PATH --> path to esp-idf directory
$ESP_IDF_PATH\export.ps1
idf.py build
```
## Run main application

```powershell
# Open Powershell
# WORKSPACE --> path to project directory
$ESP_IDF_PATH\export.ps1
cd $WORKSPACE
idf.py -p $PORT flash
idf.py -p $PORT monitor
```

## Run test cases

```powershell
# Open Powershell
$ESP_IDF_PATH\export.ps1
cd $WORKSPACE\test
idf.py -p $PORT flash
idf.py -p $PORT monitor
```