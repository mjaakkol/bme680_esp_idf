cmake_minimum_required(VERSION 3.5)

# It is assumed that BSEC asset https://www.bosch-sensortec.com/bst/products/all_products/bsec
# is directly extracted under this library as external component into directory bme680
set (EXTRA_COMPONENT_DIRS bme680)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bme680_esp)
