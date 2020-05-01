#
# Component Makefile
#

# Set exact profile name per the directory under config. This is one of the many
set (BME_PROFILE "generic_33v_3s_28d")

set (COMPONENT_ADD_INCLUDEDIRS
        "API"
        "algo/normal_version/inc"
        "examples"
        "config/${BME_PROFILE}"
        )

set (COMPONENT_SRCS
        "examples/bsec_integration.c"
        "API/bme680.c"
        "config/${BME_PROFILE}/bsec_serialized_configurations_iaq.c"
        )

register_component("config/${BME_PROFILE}")

# The integration code gives plenty of these warmings so they are dummed down in this component
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-but-set-variable -Wno-unused-variable)

# Declaring the precompiled BSEC lib as imported global so that it doesn't need to be recompiled
add_library(bme680lib STATIC IMPORTED GLOBAL)
set_target_properties(bme680lib PROPERTIES IMPORTED_LOCATION "${COMPONENT_DIR}/algo/normal_version/bin/esp32/libalgobsec.a")

# For some reason, math library from newlib isn't implicitly imported so that needs to be manually
# done for now. You might be able to get rid of the hack at some point
add_library(mathlib STATIC IMPORTED GLOBAL)
set_target_properties(mathlib PROPERTIES IMPORTED_LOCATION "${IDF_PATH}/components/newlib/lib/libm.a")

target_link_libraries(${COMPONENT_LIB} PUBLIC bme680lib mathlib)
