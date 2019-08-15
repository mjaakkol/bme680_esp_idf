#include <time.h>
#include <string.h>
#include <fastmath.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/i2c.h>
#include <bsec_integration.h>
#include <bsec_serialized_configurations_iaq.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_FREQUENCY   100000
#define I2C_GPIO_SDA    GPIO_NUM_23
#define I2C_GPIO_SCL    GPIO_NUM_22
#define ACTIVE_I2C      I2C_NUM_1

static const char* TAG = "bme680_sensor";
static const char* sensor_binary = "sensor_blob";

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to write to the bus where BME680 is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data_ptr, data_len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)ret;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to read from bus where BME680 is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    // Feeding the command in
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    //bme680_sleep(150);
    // Reading data back
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, reg_data_ptr, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data_ptr + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)ret;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
static void bme680_sleep(uint32_t t_ms)
{
    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...
    vTaskDelay(pdMS_TO_TICKS(t_ms));
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...
    ESP_LOGI(TAG, "iaq %f temp %f hum %f press %f static iaq %f co2_equivalent %f breath voc %f", iaq, temperature, humidity, pressure,static_iaq, co2_equivalent, breath_voc_equivalent);
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */

uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("state", NVS_READONLY, &my_handle);
    ESP_ERROR_CHECK( err );

    err = nvs_get_blob(my_handle, sensor_binary, state_buffer, &n_buffer);
    // We close this anyway even if the operation didn't succeed.
    nvs_close(my_handle);
    if (err == ESP_OK){
        return n_buffer;
    }
    ESP_LOGW(TAG, "loading sensor binary blob failed with code %d", err);
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("state", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK( err );

    err = nvs_set_blob(my_handle, sensor_binary, state_buffer, length);
    ESP_ERROR_CHECK( err );
    err = nvs_commit(my_handle);
    ESP_ERROR_CHECK(err);
    nvs_close(my_handle);
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    ESP_LOGI(TAG, "Loading configuration: buffer-size %d  config size %d", n_buffer, sizeof(bsec_config_iaq));
    assert(n_buffer >= sizeof(bsec_config_iaq));
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));

    return sizeof(bsec_config_iaq);
}


static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    i2c_param_config(ACTIVE_I2C, &conf);
    return i2c_driver_install(ACTIVE_I2C, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
int initialize_sensor()
{
    return_values_init ret;
    esp_err_t err = i2c_master_init();
    ESP_ERROR_CHECK( err );

    ESP_LOGI(TAG, "I2C initialized");
    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, bme680_sleep, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not initialize BME680 */
        ESP_LOGE(TAG, "initializing BME680 failed %d", ret.bme680_status);
        return (int)ret.bme680_status;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        ESP_LOGE(TAG, "initializing BSEC failed %d", ret.bsec_status);
        return (int)ret.bsec_status;
    }

    ESP_LOGI(TAG, "Entering into the loop");
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(bme680_sleep, esp_timer_get_time, output_ready, state_save, 10000);

    return 0;
}

/*! @}*/


void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    initialize_sensor();
}

/*
Current ESP-IDF 4.0 doesn't provide the function for newlib so the approximation
is provided here for __ieee754_sqrtf

Once the linker warning appears, feel free to delete the code
*/

float __ieee754_sqrtf(float number)
{
   long i;
   float x2, y;
   const float threehalfs = 1.5F;

   x2 = number * 0.5F;
   y  = number;
   i  = * ( long * ) &y;                     // floating point bit level hacking [sic]
   i  = 0x5f3759df - ( i >> 1 );             // Newton's approximation
   y  = * ( float * ) &i;
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 1st iteration
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 2nd iteration
   y  = y * ( threehalfs - ( x2 * y * y ) ); // 3rd iteration

   return 1/y;
}

