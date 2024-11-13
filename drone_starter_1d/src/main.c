#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "math.h"

#define SCL_IO_PIN 5
#define SDA_IO_PIN 4
#define ON_SWITCH_PIN 9 // Enable motor when low

#define REG_POWER_MGMT_1 0x6B

// PID constants (these need to be tuned)
#define KP 5
#define KI 1
#define KD 0.5

// PWM maximum duty for 13-bit resolution
#define PWM_MAX_DUTY 8191

void init_single_pwm(int gpio_pin_number)
{
    // Set up the PWM controller for the motor
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 200, // Hopefully something that isn't annoying to hear
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t pwm_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio_pin_number,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_config));
}

/* Duty cycle is out of 13 bits (0-8191)*/
void pwm_set_duty(int duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191-duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

int clamp(int value, int min, int max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }
    return value;
}

void app_main() {

    vTaskDelay(100); // wait for the IMU to wake up

    // Configure the I2C bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t mpu_i2c_config = {
        .scl_speed_hz = 400000,
        .device_address = 0x68
    };
    i2c_master_dev_handle_t mpu_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu_i2c_config, &mpu_handle));

    // Set POWER_MGMT_1 register to all 0 to enable sampling
    uint8_t tx[2] = {REG_POWER_MGMT_1, 0};
    i2c_master_transmit(mpu_handle, tx, 2, -1);

    tx[0] = 0x1a; //REG_CONFIG;
    tx[1] = 0x06; // Set low-pass filter to 5Hz bandwidth
    i2c_master_transmit(mpu_handle, tx, 2, -1);

    // Read from the WHOAMI register, expect 0x68 (decimal 104)
    //uint8_t reg = 0x6B;
    //uint8_t whoami = 0xFF;
    //ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_handle, &reg, 1, &whoami, 1, -1));
    //printf("whoami (register 0x6B): 0x%x\n", whoami);


    uint8_t reg = 0x3B;
    uint8_t rx[6]; // Accelerometer data

    init_single_pwm(0);

    int iteration = 0;
    float cum_error = 0;
    float last_error = 0;

    // PID variables
    float dt = 0.01; // Time interval for each control cycle (10ms)


    while(1) {
        // Read the raw accelerometer values
        reg = 0x3b;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu_handle, &reg, 1, rx, 6, -1));

        int16_t acc_x = (rx[0] << 8) + rx[1];
        int16_t acc_y = (rx[2] << 8) + rx[3];
        int16_t acc_z = (rx[4] << 8) + rx[5];


        // To calculate the angle, we just need to look at Y and Z (depending on mounting)
        // Doesn't matter what the full-scale value is as long as it's the same!
        float angle = atan2f(-acc_y, acc_z) * 57.296f; // atan2 result is radiants, convert to degrees

        float target = 0.0f; // Degrees (straight out, parallel to the ground)



        // Ok, this is where the magic happens.  Figure out what power to set the motor to!


        // Calculate PID control
        float error = target - angle;
        cum_error += error * dt;
        float rate_of_change = (error - last_error) / dt;

        // PID formula
        float control_signal = KP * error + KI * cum_error + KD * rate_of_change;

        // Convert control signal to PWM duty cycle
        int power = (int)(control_signal * PWM_MAX_DUTY / 100.0);

        // So that we don't have negative PWM
        power = clamp(power, 0, PWM_MAX_DUTY); // Clamp power within PWM range

        pwm_set_duty(power);



        // Print out data, but only once in a while
        if(iteration % 50 == 0){
            printf("X: %d, Y: %d, Z: %d   angle: %f, power: %d\n", acc_x, acc_y, acc_z, angle, power);
        }
        iteration++;

        last_error = error; // Update error for the next cycle


        vTaskDelay(1); // Sleep about 10ms, feel free to change this
    }

}
