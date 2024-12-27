/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"

/* Example code to talk to a MPU9250 MEMS accelerometer and gyroscope.
   Ignores the magnetometer, that is left as a exercise for the reader.

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore SPI) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic MPU9250 board, other
   boards may vary.

   GPIO 4 (pin 6) MISO/spi0_rx-> ADO on MPU9250 board
   GPIO 5 (pin 7) Chip select -> NCS on MPU9250 board
   GPIO 6 (pin 9) SCK/spi0_sclk -> SCL on MPU9250 board
   GPIO 7 (pin 10) MOSI/spi0_tx -> SDA on MPU9250 board
   3.3v (pin 36) -> VCC on MPU9250 board
   GND (pin 38)  -> GND on MPU9250 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
   The particular device used here uses the same pins for I2C and SPI, hence the
   using of I2C names
*/

#define PWM_PIN_1 15         // GPIO pin for PWM Motor 1
#define PWM_PIN_2 16         // GPIO pin for PWM Motor 2
#define PWM_PIN_3 17         // GPIO pin for PWM Motor 3
#define PWM_PIN_4 18         // GPIO pin for PWM Motor 4

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0
#define READ_BIT 0x80

#define INTERRUPT_PIN 22
volatile bool data_ready = false; // Flag to indicate data availability

// Global variables to store angles
static float angle_x = 0.0f; // Roll
static float angle_y = 0.0f; // Pitch

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void mpu9250_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}


static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_us(1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    sleep_us(1);
}

void mpu9250_enable_dmp() {
    uint8_t buffer[2];

    // Reset DMP
    buffer[0] = 0x6B; // PWR_MGMT_1 register
    buffer[1] = 0x00; // Set to 0 to wake up the MPU
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(100);

    // Enable DMP
    buffer[0] = 0x6A; // USER_CTRL register
    buffer[1] = 0x80; // Enable DMP
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(100);
}



static void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14]; // Buffer to read all 14 bytes of data in one go
    read_registers(0x3B, buffer, 14); // Start reading from accelerometer X high byte

    // Parse accelerometer data
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
    }

    // Parse temperature data
    *temp = (buffer[6] << 8) | buffer[7];

    // Parse gyroscope data
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1];
    }
}

void mpu9250_enable_interrupt() {
    uint8_t buffer[2];

    // Enable interrupt on INT pin (active high)
    buffer[0] = 0x37;  // INT_PIN_CFG register
    buffer[1] = 0x10;  // LATCH_INT_EN = 0, INT_ANYRD_2CLEAR = 1
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();

    // Enable Data Ready interrupt
    buffer[0] = 0x38;  // INT_ENABLE register
    buffer[1] = 0x01;  // DATA_RDY_EN = 1
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
}
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == INTERRUPT_PIN) {
        uint8_t int_status;
        read_registers(0x3A, &int_status, 1); // INT_STATUS register
        if (int_status & 0x02) { // DMP_INT triggered
            data_ready = true;
            printf("DMP interrupt detected.\n");
    }
}}



void configure_gpio_interrupt() {
    gpio_set_function(INTERRUPT_PIN, GPIO_FUNC_SIO); // Configure as GPIO
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);           // Set as input
    gpio_pull_up(INTERRUPT_PIN);                    // Enable internal pull-up resistor
    gpio_set_irq_enabled_with_callback(INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE, true, gpio_callback);
}

void dump_fifo() {
    // Read the FIFO count
    uint8_t fifo_count[2];
    read_registers(0x72, fifo_count, 2); // FIFO_COUNT_H and FIFO_COUNT_L
    uint16_t count = (fifo_count[0] << 8) | fifo_count[1];

    if (count > 0) {
        printf("Dumping FIFO: %d bytes\n", count);

        // Read and discard all FIFO data
        uint8_t buffer[512]; // Max FIFO size is 512 bytes
        while (count > 0) {
            // Read in chunks of up to 512 bytes or remaining count
            uint16_t bytes_to_read = (count > sizeof(buffer)) ? sizeof(buffer) : count;
            read_registers(0x74, buffer, bytes_to_read); // FIFO_R/W register
            count -= bytes_to_read;
        }
        printf("FIFO dumped.\n");
    } else {
        printf("FIFO is already empty.\n");
    }
}

float constrain(float value, float min, float max) {
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}


void mpu9250_configure_quaternion_output() {
    uint8_t buffer[2];

    // Enable FIFO
    buffer[0] = 0x6A;  // USER_CTRL register
    buffer[1] = 0x40;  // Enable FIFO
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();

    // Set FIFO to include quaternion data
    buffer[0] = 0x23;  // FIFO_EN register
    buffer[1] = 0xF0;  // Enable quaternion data (and possibly other data)
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
}
float q0, q1, q2, q3;
float yr, pr, rr, yr_angle, pr_angle, rr_angle;

void mpu9250_read_quaternion(float *q0, float *q1, float *q2, float *q3) {
    
    uint8_t fifo_count[2];
    uint8_t fifo_data[16]; // 16 bytes for quaternion data (4 x 4 bytes)

    // Read FIFO count
    read_registers(0x72, fifo_count, 2); // FIFO_COUNT_H and FIFO_COUNT_L
    uint16_t count = (fifo_count[0] << 8) | fifo_count[1];

    while(count < 16) { //insert recursive here until count is above 15 and then read the quaternions
        // Read quaternion data from FIFO
        read_registers(0x74, fifo_data, 16); // FIFO_R/W register

        // Parse quaternion data (16-bit integers)
        int16_t q0_raw = (fifo_data[0] << 8) | fifo_data[1];
        int16_t q1_raw = (fifo_data[4] << 8) | fifo_data[5];
        int16_t q2_raw = (fifo_data[8] << 8) | fifo_data[9];
        int16_t q3_raw = (fifo_data[12] << 8) | fifo_data[13];
        printf("Raw Quaternion: q0=%d, q1=%d, q2=%d, q3=%d\n", q0_raw, q1_raw, q2_raw, q3_raw);
        // Convert to floating-point numbers (scale factor depends on DMP configuration)
        *q0 = (float)q0_raw / (1 << 30);
        *q1 = (float)q1_raw / (1 << 30);
        *q2 = (float)q2_raw / (1 << 30);
        *q3 = (float)q3_raw / (1 << 30);
        printf("Scaled Quaternion: q0=%.6f, q1=%.6f, q2=%.6f, q3=%.6f\n", *q0, *q1, *q2, *q3);

        yr = -atan2(2 * (-(*q1) * (*q2) + (*q0) * (*q3)), (*q2) * (*q2) + (*q3) * (*q3) - (*q1) * (*q1) - (*q0) * (*q0));
        pr = asin(2 * ((*q2) * (*q3) + (*q0) * (*q1)));
        rr = atan2(2 * (-(*q1) * (*q3) + (*q2) * (*q0)), (*q3) * (*q3) - (*q2) * (*q2) - (*q1) * (*q1) + (*q0) * (*q0));
        yr_angle = yr * 180/M_PI;
        pr_angle = pr * 180/M_PI;
        rr_angle = rr * 180/M_PI;
        printf("angles: yaw_rateangle=%.6f, pitch_rateangle=%.6f, roll_rateangle=%.6f\n", yr_angle, pr_angle, rr_angle);

        // Normalize quaternion
    float magnitude = sqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
        if (magnitude != 0.0f) {
            *q0 /= magnitude;
            *q1 /= magnitude;
            *q2 /= magnitude;
            *q3 /= magnitude;
        }
        printf("Normalized Quaternion: q0=%.6f, q1=%.6f, q2=%.6f, q3=%.6f\n", *q0, *q1, *q2, *q3);
    } else {
        printf("Not enough data in FIFO (count=%d)\n", count);
    }
}

float PIDReturn[3] = {0, 0, 0};
float PRateRoll = 0.6;
float PRatePitch = 0.6;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = 3.5;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = 0.03;
float DRateYaw = 0;

float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw, DesiredAngleRoll, DesiredAnglePitch;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw, ErrorAngleRoll, ErrorAnglePitch;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PAngleRoll = 2; 
float PAnglePitch = 2;
float IAngleRoll = 0; 
float IAnglePitch = 0;
float DAngleRoll = 0; 
float DAnglePitch = 0;

// Define the variables for roll and pitch angles and uncertainties
float KalmanAngleRoll = 0.0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0.0, KalmanUncertaintyAnglePitch = 2*2;
float accel_roll, accel_pitch;
float gyro_rate_roll, gyro_rate_pitch, gyro_rate_yaw;

// Output array for the Kalman filter
float Kalman1DOutput[2] = {0.0, 0.0};

// PID loop variables
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Function to calculate the predicted angle and uncertainty using the Kalman equations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    // Predict step
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4.0 * 4.0;

    // Update step
    float KalmanGain = KalmanUncertainty / (1 * KalmanUncertainty + 3 + 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1.0 - KalmanGain) * KalmanUncertainty;

    // Store the results in the output array
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
    float Pterm = P * Error;
    float Iterm = PrevIterm +  I * (Error + PrevError) * 0.004/2;
    Iterm = constrain(Iterm, -400, 400);
    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    PIDOutput = constrain(PIDOutput, -400, 400);
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Iterm;
    PIDReturn[2] = Error;
}

// Reset PID
void reset_pid(void) {
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
}


// Kalman filter structure
typedef struct {
    float angle;
    float bias;
    float rate;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} KalmanFilter;

KalmanFilter kalmanX, kalmanY, kalmanZ;

// Kalman filter functions
void kalman_init(KalmanFilter *kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->P[0][0] = 0.1f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.1f;
    kf->Q_angle = 0.005f;
    kf->Q_bias = 0.001f;
    kf->R_measure = 0.03f;
}

float kalman_update(KalmanFilter *kf, float new_angle, float new_rate, float dt) {
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    float y = new_angle - kf->angle;
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
int16_t gyro_offset[3] = {0, 0, 0};
int16_t accel_offset[3] = {0, 0, 0};

void update_angles(int16_t accel[3], int16_t gyro[3], float dt) {
    printf("Raw Accel: X=%d, Y=%d, Z=%d\n", accel[0], accel[1], accel[2]);
    printf("Raw Gyro: X=%d, Y=%d, Z=%d\n", gyro[0], gyro[1], gyro[2]);
    for (int i = 0; i < 3; i++) {
        gyro[i] -= gyro_offset[i];
        accel[i] -= accel_offset[i];
    }
    printf("Accel with offset applied: X=%d, Y=%d, Z=%d\n", accel[0], accel[1], accel[2]);

    // Gyroscope rate conversion (dps to degrees)
    float gyro_rate_x = gyro[0] / 65.5f; // Roll rate
    float gyro_rate_y = gyro[1] / 65.5f; // Pitch rate
    float gyro_rate_z = gyro[2] / 65.5f; //yaw rate

    float accel_x = accel[0] / 16384.0f;
    float accel_y = accel[1] / 16384.0f;
    float accel_z = accel[2] / 16384.0f;
    printf("Scaled Accel: X=%f, Y=%f, Z=%f\n", accel_x, accel_y, accel_z);

    // Accelerometer-based angle calculation
    accel_roll = atan2f((float)accel[1],(sqrt((float)(accel_x * accel_x + accel_z * accel_z)))) * 180/M_PI; //Roll
    accel_pitch = -atan2f((float)accel[0],(sqrtf((float)(accel_y * accel_y + accel_z * accel_z)))) * 180 / M_PI;  //Pitch

    printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f\n", accel[0], accel[1], accel[2], gyro_rate_x, gyro_rate_y, gyro_rate_z);



    // Integrate gyroscope data to predict angles
    //angle_x += gyro_rate_x * dt; // Predicted roll angle
    //angle_y += gyro_rate_y * dt; // Predicted pitch angle

    // Complementary filter to combine accelerometer and gyroscope data
    //const float alpha = 0.98f; // Complementary filter coefficient
    //angle_x = alpha * angle_x + (1 - alpha) * accel_roll; // Roll angle
    //angle_y = alpha * angle_y + (1 - alpha) * accel_pitch; // Pitch angle

    // Normalize the angles to ensure they remain within a valid range
    //angle_x = normalize_angle(angle_x);
    //angle_y = normalize_angle(angle_y);
}

void kalman_filter_update(int16_t accel[3], int16_t gyro[3], float dt, float *angles) {
    float angle_accel_x = atan2f((float)-accel[0], sqrtf((float)(accel[1] * accel[1] + accel[2] * accel[2]))) * 180.0f / M_PI;  // Roll
    float angle_accel_y = atan2f((float)accel[1], sqrtf((float)(accel[0] * accel[0] + accel[2] * accel[2]))) * 180.0f / M_PI; // Pitch

    gyro_rate_roll = (float)gyro[0] / 65.5;
    gyro_rate_pitch = (float)gyro[1] / 65.5;
    gyro_rate_yaw = (float)gyro[2] / 65.5;

    angles[0] = kalman_update(&kalmanX, angle_accel_x, gyro_rate_roll, dt);  // Pitch
    angles[1] = kalman_update(&kalmanY, angle_accel_y, gyro_rate_pitch, dt);  // Roll
}

void calibrate_sensors(int16_t gyro_offset[3], int16_t accel_offset[3]) {
    const int calibration_samples = 1000; // Increase samples for better accuracy
    int32_t gyro_sum[3] = {0, 0, 0};
    int32_t accel_sum[3] = {0, 0, 0};

    int16_t gyro[3], accel[3], temp;

    printf("Calibrating sensors... Please keep the device stationary.\n");

    for (int i = 0; i < calibration_samples; i++) {
        if (data_ready) {
            mpu9250_read_raw(accel, gyro, &temp);
            

        for (int j = 0; j < 3; j++) {
            gyro_sum[j] += gyro[j];
            accel_sum[j] += accel[j];
        }
        data_ready=false;
        }
        sleep_ms(5); // Small delay between samples
    }

    for (int j = 0; j < 3; j++) {
        gyro_offset[j] = gyro_sum[j] / calibration_samples;
        accel_offset[j] = accel_sum[j] / calibration_samples;
    }
    accel_offset[2] -= 16384; // Assuming 1g = 16384 LSB

    printf("Gyro offsets: X=%d, Y=%d, Z=%d\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    printf("Accel offsets: X=%d, Y=%d, Z=%d\n", accel_offset[0], accel_offset[1], accel_offset[2]);
}
void pwm_init_motor(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice_num, 64.0f); // Adjust clock divider for a 50 Hz frequency
    pwm_set_wrap(slice_num, 20000);   // Set period to 20 ms (20000 ticks at 1 MHz base frequency)
    pwm_set_enabled(slice_num, true);
}
void calibrate_escs() {
    printf("Calibrating ESCs...\n");
    
    // Set all ESC signals to max throttle
    uint16_t max_throttle = 2000;
    pwm_set_gpio_level(PWM_PIN_1, (max_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_2, (max_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_3, (max_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_4, (max_throttle - 1000) * 125 / 1000);

    sleep_ms(2000); // Wait 2 seconds for ESCs to register max throttle

    // Set all ESC signals to min throttle
    uint16_t min_throttle = 1000;
    pwm_set_gpio_level(PWM_PIN_1, (min_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_2, (min_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_3, (min_throttle - 1000) * 125 / 1000);
    pwm_set_gpio_level(PWM_PIN_4, (min_throttle - 1000) * 125 / 1000);

    sleep_ms(2000); // Wait 2 seconds for ESCs to register min throttle

    printf("ESC calibration complete.\n");
}


void check_interrupt_pin_state() {
    
    printf("Interrupt pin state: %d\n", data_ready);
}

int main() {
    stdio_init_all();
    pwm_init_motor(PWM_PIN_1);
    pwm_init_motor(PWM_PIN_2);
    pwm_init_motor(PWM_PIN_3);
    pwm_init_motor(PWM_PIN_4);
    calibrate_escs();

    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

    gpio_init(INTERRUPT_PIN);                              // Initialize GPIO for interrupt
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);                  // Set as input
    gpio_pull_up(INTERRUPT_PIN);                           // Enable pull-up resistor

    mpu9250_reset();
    mpu9250_enable_dmp();
    mpu9250_configure_quaternion_output();
    configure_gpio_interrupt();                            // Attach interrupt and callback
    mpu9250_enable_interrupt();                            // Enable interrupt on MPU9250



 
    kalman_init(&kalmanX);
    kalman_init(&kalmanY);
    kalman_init(&kalmanZ);
    int16_t acceleration[3], gyro[3], temp;
    sleep_ms(1000);
    
    //calibrate_sensors(gyro_offset, accel_offset);
    uint64_t prev_time = to_us_since_boot(get_absolute_time());
    float angles[3]; // Filtered angles: [pitch, roll, yaw]
    InputThrottle=1500.0f; // base throttle

    

    while (1) {
        printf("data_ready flag state: %d\n", data_ready);
        if (data_ready) {
            printf("Reading quaternion...\n");
            mpu9250_read_quaternion(&q0, &q1, &q2, &q3);
            data_ready = false;
        }
        uint64_t current_time = to_us_since_boot(get_absolute_time());
        float dt = (current_time - prev_time) / 1e6f; // Convert to seconds
        prev_time = current_time;
        /*update_angles(acceleration, gyro, dt);
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gyro_rate_roll, accel_roll);
        KalmanAngleRoll = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyro_rate_pitch, accel_pitch);
        KalmanAnglePitch = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch = Kalman1DOutput[1];


        printf("Roll %f, Pitch %f\n", accel_roll, accel_pitch);
        printf("KalmanRoll %f, KalmanPitch %f\n", KalmanAngleRoll, KalmanAnglePitch);
        DesiredAngleRoll=0.0f;
        DesiredAnglePitch=0.0f;

        
        char ch = getchar_timeout_us(0);
        
        if (ch == ' ') {
            
            InputThrottle += 10;  // Increase duty cycle

            printf("Increased PWM duty: %f\n", InputThrottle);
        }
        else if(ch == 0x1B) { // Assuming shift sends escape character

            InputThrottle -= 10;  // Decrease duty cycle

            printf("Decreased PWM duty: %f\n", InputThrottle);
        } 
        
        DesiredRateYaw=0.0f;

        
        
        ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
        ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

        pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
        DesiredRateRoll = PIDReturn[0];
        PrevErrorAngleRoll = PIDReturn[1];
        PrevItermAngleRoll = PIDReturn[2];

        pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
        DesiredRatePitch=PIDReturn[0];
        PrevErrorAnglePitch=PIDReturn[1];
        PrevItermAnglePitch=PIDReturn[2];

        ErrorRateRoll = DesiredRateRoll - gyro_rate_roll;
        ErrorRatePitch = DesiredRatePitch - gyro_rate_pitch;
        ErrorRateYaw = DesiredRateYaw - gyro_rate_yaw;

        pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll = PIDReturn[0];
        PrevErrorRateRoll = PIDReturn[1];
        PrevItermRateRoll = PIDReturn[2];

        pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch = PIDReturn[0];
        PrevErrorRatePitch = PIDReturn[1];
        PrevItermRateRoll = PIDReturn[2];

        pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw = PIDReturn[0];
        PrevErrorRateYaw = PIDReturn[1];
        PrevItermRateYaw = PIDReturn[2];
        if(InputThrottle > 1800)
            InputThrottle = 1800;
        
        MotorInput1 = 1.024 * (InputThrottle-InputRoll-InputPitch-InputYaw);
        MotorInput2 = 1.024 * (InputThrottle-InputRoll+InputPitch+InputYaw);
        MotorInput3 = 1.024 * (InputThrottle+InputRoll+InputPitch-InputYaw);
        MotorInput4 = 1.024 * (InputThrottle+InputRoll-InputPitch+InputYaw);
        */
        if (MotorInput1 >2000)
            MotorInput1 = 1999;
        if (MotorInput2 >2000)
            MotorInput2 = 1999;
        if (MotorInput3 >2000)
            MotorInput3 = 1999;
        if (MotorInput4 >2000)
            MotorInput4 = 1999;
        
        int ThrottleIdle = 1180;
        if(MotorInput1 < ThrottleIdle)
            MotorInput1 = ThrottleIdle;
        if(MotorInput2 < ThrottleIdle)
            MotorInput2 = ThrottleIdle;
        if(MotorInput3 < ThrottleIdle)
            MotorInput3 = ThrottleIdle;
        if(MotorInput4 < ThrottleIdle)
            MotorInput4 = ThrottleIdle; 
        
        int ThrottleCutOff = 1000;

        if (InputThrottle < 1050) {
            MotorInput1 = ThrottleCutOff;
            MotorInput2 = ThrottleCutOff;
            MotorInput3 = ThrottleCutOff;
            MotorInput4 = ThrottleCutOff;
            reset_pid();
        }           
        pwm_set_gpio_level(PWM_PIN_1, MotorInput1);
        pwm_set_gpio_level(PWM_PIN_2, MotorInput2);
        pwm_set_gpio_level(PWM_PIN_3, MotorInput3);
        pwm_set_gpio_level(PWM_PIN_4, MotorInput4);

        check_interrupt_pin_state();

        
        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
  

        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.
        // Note this is chip temperature.
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        
        sleep_ms(100);
    }
}