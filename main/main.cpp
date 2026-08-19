#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "mpu6050/mpu6050.hpp"
#include "espidf.hpp"

#include <math.h>

static constexpr char TAG[] = "main";
static constexpr gpio_num_t SDA1 = GPIO_NUM_8;
static constexpr gpio_num_t SCL1 = GPIO_NUM_9;
static constexpr uint32_t FREQ1 = 4000000;

static constexpr gpio_num_t SDA2 = GPIO_NUM_4;
static constexpr gpio_num_t SCL2 = GPIO_NUM_5;
static constexpr uint32_t FREQ2 = 4000000;

constexpr float RAD_TO_DEG = 57.2957795f;

EspIdfDelay time;

const float alpha = 0.8; // smaller = smoother but slower
float ax1Filt = 0, ay1Filt = 0, az1Filt = 0;
float ax2Filt = 0, ay2Filt = 0, az2Filt = 0;

bool filterInitialized = false;

void printRawValues(MPU6050 imu)
{
    Vector3i16 accel;
    Vector3i16 gyro;

    Error accel_err = imu.readRawAccel(accel);
    Error gyro_err = imu.readRawGyro(gyro);

    if (accel_err == Error::OK && gyro_err == Error::OK)
    {
        ESP_LOGI(TAG,
                 "Gyro: %d, %d, %d | Accel: %d, %d, %d",
                 gyro.x, gyro.y, gyro.z,
                 accel.x, accel.y, accel.z);
    }
    else
    {
        ESP_LOGW(TAG, "MPU6050 read failed");
    }

    time.delayMs(100);
}

void printValues(MPU6050 imu, const char tag[])
{
    Vector3f accel;
    Vector3f gyro;

    Error accel_err = imu.readAccel(accel);
    Error gyro_err = imu.readGyro(gyro);

    if (accel_err == Error::OK && gyro_err == Error::OK)
    {
        ESP_LOGI(tag,
                 "Accel(g): %.2f, %.2f, %.2f | Gyro(dps): %.2f, %.2f, %.2f",
                 accel.x, accel.y, accel.z,
                 gyro.x, gyro.y, gyro.z);
    }
    else
    {
        ESP_LOGW(tag, "MPU6050 read failed");
    }

    time.delayMs(100);
}

float linkAngleDeg(float ax, float az)
{
    return atan2(-ax, -az) * RAD_TO_DEG;
}

void updateLowPassFilters(
    float ax1, float ay1, float az1,
    float ax2, float ay2, float az2)
{
    // First sample initializes filters; prevents a slow ramp-up from zero.
    if (!filterInitialized)
    {
        ax1Filt = ax1;
        ay1Filt = ay1;
        az1Filt = az1;
        ax2Filt = ax2;
        ay2Filt = ay2;
        az2Filt = az2;
        filterInitialized = true;
        return;
    }
    ax1Filt += alpha * (ax1 - ax1Filt);
    ay1Filt += alpha * (ay1 - ay1Filt);
    az1Filt += alpha * (az1 - az1Filt);

    ax2Filt += alpha * (ax2 - ax2Filt);
    ay2Filt += alpha * (ay2 - ay2Filt);
    az2Filt += alpha * (az2 - az2Filt);
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    i2c_master_bus_config_t bus_config_1 = {};
    bus_config_1.i2c_port = I2C_NUM_0;
    bus_config_1.sda_io_num = SDA1;
    bus_config_1.scl_io_num = SCL1;
    bus_config_1.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config_1.glitch_ignore_cnt = 7;
    bus_config_1.flags.enable_internal_pullup = true;

    i2c_master_bus_config_t bus_config_2 = {};
    bus_config_2.i2c_port = I2C_NUM_1;
    bus_config_2.sda_io_num = SDA2;
    bus_config_2.scl_io_num = SCL2;
    bus_config_2.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config_2.glitch_ignore_cnt = 7;
    bus_config_2.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle_1 = nullptr;
    i2c_master_bus_handle_t bus_handle_2 = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_1, &bus_handle_1));
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_2, &bus_handle_2));

    EspIdfI2C bus1(bus_handle_1);
    EspIdfI2C bus2(bus_handle_2);
    MPU6050 link1(bus1);
    MPU6050 ref(bus1, 0x69);
    MPU6050 link2(bus2);

    ESP_LOGI(TAG, "IMU bus1 ready");

    Error err = link1.begin();

    if (err != Error::OK)
    {
        ESP_LOGE(TAG, "Link 1 init failed");
        return; // Stops app_main; the device remains idle.
    }

    ESP_LOGI(TAG, "MPU6050 init success");

    Vector3f accel1, accel2;

    Error accel1_err = link1.readAccel(accel1);
    Error ref_err = ref.readAccel(accel2);

    updateLowPassFilters(accel1.x, accel1.y, accel1.z, accel2.x, accel2.y, accel2.z);

    float angleLink1 = linkAngleDeg(ax1Filt, az1Filt);
    float angleLink2 = linkAngleDeg(ax2Filt, az2Filt);

    float zeroOffset = angleLink2 - angleLink1;

    while (true)
    {

        Error accel1_err = link1.readAccel(accel1);
        Error ref_err = link2.readAccel(accel2);

        updateLowPassFilters(accel1.x, accel1.y, accel1.z, accel2.x, accel2.y, accel2.z);

        float angleLink1 = linkAngleDeg(ax1Filt, az1Filt);
        float angleLink2 = linkAngleDeg(ax2Filt, az2Filt);
        float jointAngle = angleLink2 - angleLink1 - zeroOffset;

        if (jointAngle > 180)
            jointAngle -= 360;
        if (jointAngle < -180)
            jointAngle += 360;

        ESP_LOGI("Joint Angle test", "Joint Angle: ``%.2f", jointAngle);

        time.delayMs(100);

        // printValues(link1,"Link 1");
        // printValues(ref,"Ref Link");
        // printValues(link2,"Link 2");1
        // printRawValues(link1);
    }
}