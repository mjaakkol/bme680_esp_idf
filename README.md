# BME680 ESP32-IDF v4.0
BME680 software build on top of ESP-IDF 4.0 generation and Bosch BSEC stack

## Introduction
This repository implements BSEC APIs for BME680 integrating into ESP32 IDF 4.0 based CMake build system. This implements the example APIs and should provide simple starting point for those who want to use ESP32-IDF development environment and Bosch library assets.

The library with algorithms can be loaded from https://www.bosch-sensortec.com/bst/products/all_products/bsec.

The only tweak beyond the example is the usage of BME680_I2C_ADDR_PRIMARY and BME680_I2C_ADDR_SECONDARY that you need to adjust manually in bsec_integration.c file. Different boards can have different address so you want to check the board manual for the details.

The implementation has been tested against ESP32-IDF v4.0 Master repository and Bosch Sensortec Environmental Cluster (BSEC) Software
v1.4.7.4 | July 3rd, 2019.

## Integration
1. Clone the repository into the target directory
2. Exract BSEC software into bme680 directly so that the folder structure is directly under the directory (eg. API, algo, example should be directly under bme680). CMakefileLists.txt should already be there.
3. Configure the desired profile for the compilation by modifying bme680/CMakeLists.txt file and matching the profile name with the directory under config-directory)
4. Make sure that you have nvs storage partition in your system and then type commands:
```
idf.py menuconfig
idf.py flash monitor
```
