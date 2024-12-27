#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"

#define PWM_PIN_1 15         // GPIO pin for PWM Motor 1
#define PWM_PIN_2 16         // GPIO pin for PWM Motor 2
#define PWM_PIN_3 17         // GPIO pin for PWM Motor 3
#define PWM_PIN_4 18         // GPIO pin for PWM Motor 4

// Pin configuration for the MPU9250 SPI interface
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

// I2C pins for the magnetometer
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

#define SPI_PORT spi0
#define READ_BIT 0x80
#define MAG_ADDRESS 0x0C //magnetometer add
#define MAG_CNTL1 0x0A //cntrl reg 1
#define MAG_MODE 0x16 //continuous measurement mode
#define MAG_HXL 0x03 //magnetometer data reg start

#define INTERRUPT_PIN 22
volatile bool data_ready = false; // Flag to indicate data availability


// Define I2C Pins and Address
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21
#define MS5611_ADDR 0x76


// Calibration offsets
int16_t gyro_offset[3] = {0, 0, 0};
int16_t accel_offset[3] = {0, 0, 0};
// Global variables
uint16_t C1, C2, C3, C4, C5, C6; // Calibration data
int64_t dT, TEMP, OFF, SENS, T2, OFF2, SENS2;
float pressure, cTemp, fTemp;


// Global Variables for Double Buffering
typedef struct {
    uint32_t D1, D2; // Raw pressure and temperature values
    int16_t accel[3], gyro[3], temp;
    float dt;
} SensorBuffer;

volatile SensorBuffer front_buffer; // Buffer for Core 0 to read
volatile SensorBuffer back_buffer;  // Buffer for Core 1 to update
volatile bool new_data_ready = false; // Flag to signal buffer swap
// Mutex for Synchronizing Buffer Access
mutex_t buffer_mutex;

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
    kf->bias = 0.000f;
    kf->rate = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->Q_angle = 0.01f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.1f;
}

float kalman_update(KalmanFilter *kf, float new_angle, float new_rate, float dt) {
    // Predict phase
    kf->rate = new_rate - kf->bias; // Remove bias from the rate
    kf->angle += dt * kf->rate;     // Predict the new angle based on the rate

    // Update error covariance matrix
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // Update phase
    float y = new_angle - kf->angle;  // Innovation (angle difference)
    float S = kf->P[0][0] + kf->R_measure; // Innovation covariance
    float K[2]; // Kalman gain
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    kf->angle += K[0] * y; // Update the angle with the innovation
    kf->bias += K[1] * y;  // Update the bias with the innovation

    // Update error covariance matrix
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle; // Return the updated angle
}

typedef struct {
    float altitude;  // Estimated altitude
    float velocity;  // Estimated vertical velocity
    float P[2][2];   // Error covariance matrix
    float Q[2][2];   // Process noise covariance
    float R;         // Measurement noise covariance
} AltitudeKalmanFilter;

void altitude_kalman_init(AltitudeKalmanFilter *kf) {
    kf->altitude = 0.0f;
    kf->velocity = 0.0f;

    // Initialize covariance matrices
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    // Process noise covariance
    kf->Q[0][0] = 0.1f;
    kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;
    kf->Q[1][1] = 0.1f;

    // Measurement noise covariance
    kf->R = 1.0f;
}

void altitude_kalman_update(AltitudeKalmanFilter *kf, float measured_altitude, float dt) {
    // State transition matrix (A)
    float A[2][2] = {
        {1.0f, dt},
        {0.0f, 1.0f}
    };

    // Measurement matrix (H)
    float H[2] = {1.0f, 0.0f};

    // Predicted state
    float predicted_altitude = kf->altitude + kf->velocity * dt;
    float predicted_velocity = kf->velocity;

    // Predicted error covariance
    float P_pred[2][2];
    P_pred[0][0] = A[0][0] * kf->P[0][0] + A[0][1] * kf->P[1][0] + kf->Q[0][0];
    P_pred[0][1] = A[0][0] * kf->P[0][1] + A[0][1] * kf->P[1][1] + kf->Q[0][1];
    P_pred[1][0] = A[1][0] * kf->P[0][0] + A[1][1] * kf->P[1][0] + kf->Q[1][0];
    P_pred[1][1] = A[1][0] * kf->P[0][1] + A[1][1] * kf->P[1][1] + kf->Q[1][1];

    // Measurement residual
    float y = measured_altitude - predicted_altitude;

    // Residual covariance
    float S = P_pred[0][0] + kf->R;

    // Kalman gain
    float K[2];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[1][0] / S;

    // Update state estimate
    kf->altitude = predicted_altitude + K[0] * y;
    kf->velocity = predicted_velocity + K[1] * y;

    // Update error covariance
    kf->P[0][0] = (1.0f - K[0] * H[0]) * P_pred[0][0];
    kf->P[0][1] = (1.0f - K[0] * H[0]) * P_pred[0][1];
    kf->P[1][0] = -K[1] * H[0] * P_pred[0][0] + P_pred[1][0];
    kf->P[1][1] = -K[1] * H[0] * P_pred[0][1] + P_pred[1][1];
}


// SPI and MPU9250 helper functions
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
    uint8_t buf[] = {0x6B, 0x00};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    sleep_us(50);
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

// Magnetometer functions
void i2c_init_magnetometer() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

void mpu9250_enable_magnetometer() {
    uint8_t buffer[2];

    // Enable I2C bypass mode to directly access the magnetometer
    buffer[0] = 0x37;  // INT_PIN_CFG register
    buffer[1] = 0x02;  // Enable bypass mode
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();

    sleep_ms(10);

    // Initialize the magnetometer in continuous measurement mode
    buffer[0] = MAG_CNTL1;
    buffer[1] = MAG_MODE;
    i2c_write_blocking(i2c_default, MAG_ADDRESS, buffer, 2, false);
    sleep_ms(10);
}


void read_magnetometer(int16_t *mag) {
    uint8_t raw_data[6];

    i2c_write_blocking(i2c_default, MAG_ADDRESS, MAG_HXL, 1, true);
    i2c_read_blocking(i2c_default, MAG_ADDRESS, raw_data, 6, false);

    mag[0] = (raw_data[1] << 8) | raw_data[0];
    mag[1] = (raw_data[3] << 8) | raw_data[2];
    mag[2] = (raw_data[5] << 8) | raw_data[4];
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
        data_ready = true;
    }
}
void configure_gpio_interrupt() {
    gpio_set_function(INTERRUPT_PIN, GPIO_FUNC_SIO); // Configure as GPIO
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);           // Set as input
    gpio_pull_up(INTERRUPT_PIN);                    // Enable internal pull-up resistor
    gpio_set_irq_enabled_with_callback(
        INTERRUPT_PIN, GPIO_IRQ_EDGE_RISE, true, gpio_callback);
}

// Function to initialize I2C
void init_i2c() {
    i2c_init(I2C_PORT, 100000); // Initialize I2C at 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Function to write a single command to the sensor
void i2c_write_byte(uint8_t address, uint8_t command) {
    int result = i2c_write_blocking(I2C_PORT, address, &command, 1, false);
    if (result < 0) {
        printf("I2C write error: %d\n", result);
    }
}

// Function to read multiple bytes from the sensor
void i2c_read_bytes(uint8_t address, uint8_t command, uint8_t *buffer, size_t length) {
    i2c_write_blocking(I2C_PORT, address, &command, 1, true); // Send command
    i2c_read_blocking(I2C_PORT, address, buffer, length, false); // Read data
}

// Function to read calibration data
void read_calibration_data() {
    uint8_t data[2];

    i2c_read_bytes(MS5611_ADDR, 0xA2, data, 2);
    C1 = data[0] << 8 | data[1]; // Pressure sensitivity

    i2c_read_bytes(MS5611_ADDR, 0xA4, data, 2);
    C2 = data[0] << 8 | data[1]; // Pressure offset

    i2c_read_bytes(MS5611_ADDR, 0xA6, data, 2);
    C3 = data[0] << 8 | data[1]; // Temp coefficient of pressure sensitivity

    i2c_read_bytes(MS5611_ADDR, 0xA8, data, 2);
    C4 = data[0] << 8 | data[1]; // Temp coefficient of pressure offset

    i2c_read_bytes(MS5611_ADDR, 0xAA, data, 2);
    C5 = data[0] << 8 | data[1]; // Reference temperature

    i2c_read_bytes(MS5611_ADDR, 0xAC, data, 2);
    C6 = data[0] << 8 | data[1]; // Temp coefficient of the temperature
}

// Function to read raw pressure or temperature
void write_raw_value(uint8_t command) {
    i2c_write_byte(MS5611_ADDR, command); // Start conversion

}

uint32_t read_raw_value(uint8_t command) {
    uint8_t data[3];
    int result = i2c_read_blocking(I2C_PORT, MS5611_ADDR, data, 3, false);
    if (result < 0) {
        printf("I2C read error: %d\n", result);
    }
}




// Function to calculate temperature and pressure
void calculate_temperature_and_pressure() {
    dT = front_buffer.D2 - ((int64_t)C5 << 8);
    TEMP = 2000 + (dT * C6) / 8388608;

    OFF = ((int64_t)C2 << 16) + (C4 * dT) / 128;
    SENS = ((int64_t)C1 << 15) + (C3 * dT) / 256;

    // Second-order temperature compensation
    if (TEMP < 2000) {
        T2 = (dT * dT) / 2147483648;
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;

        if (TEMP < -1500) {
            OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }
    } else {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    pressure = ((((front_buffer.D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0;
    cTemp = TEMP / 100.0;
    fTemp = cTemp * 1.8 + 32;
}


float calculate_altitude(uint32_t pressure_adc) {
    const float sea_level_pressure = 101325.0f; // Pascals
    return 44330.0f * (1.0f - powf(pressure_adc / sea_level_pressure, 0.1903f));
}


void kalman_filter_update(int16_t accel[3], int16_t gyro[3], float dt, float *angles) {
    float angle_accel_x = atan2f((float)accel[1], (float)accel[2]) * 180 / M_PI;
    float angle_accel_y = atan2f((float)accel[0], (float)accel[2]) * 180 / M_PI;

    float gyro_rate_x = (float)gyro[0] / 65.5;
    float gyro_rate_y = (float)gyro[1] / 65.5;
    float gyro_rate_z = (float)gyro[2] / 65.5;

    angles[0] = kalman_update(&kalmanX, angle_accel_x, gyro_rate_x, dt);  // X-axis (Pitch)
    angles[1] = kalman_update(&kalmanY, angle_accel_y, gyro_rate_y, dt);  // Y-axis (Roll)
    angles[2] = kalman_update(&kalmanZ, 0, gyro_rate_z, dt);              // Z-axis (Yaw)
}

// PID structure
typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
} PID;

PID pid_pitch, pid_roll, pid_yaw;

// Initialize PID controller
void pid_init(PID *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

// PID compute function
float pid_compute(PID *pid, float target, float current, float dt) {
    float error = target - current;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

void pwm_init_motor(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice_num, 64.0f); // Adjust clock divider for a 50 Hz frequency
    pwm_set_wrap(slice_num, 20000);   // Set period to 20 ms (20000 ticks at 1 MHz base frequency)
    pwm_set_enabled(slice_num, true);
}

void calibrate_sensors(int16_t gyro_offset[3], int16_t accel_offset[3]) {
    const int calibration_samples = 500; // Number of samples for calibration
    int32_t gyro_sum[3] = {0, 0, 0};
    int32_t accel_sum[3] = {0, 0, 0};

    int16_t gyro[3], accel[3], temp;

    printf("Calibrating sensors... Please keep the device stationary.\n");

    for (int i = 0; i < calibration_samples; i++) {
        mpu9250_read_raw(accel, gyro, &temp);

        for (int j = 0; j < 3; j++) {
            gyro_sum[j] += gyro[j];
            accel_sum[j] += accel[j];
        }

        sleep_ms(5); // Delay between samples
    }

    // Calculate average offsets
    for (int j = 0; j < 3; j++) {
        gyro_offset[j] = gyro_sum[j] / calibration_samples;
        accel_offset[j] = accel_sum[j] / calibration_samples;
    }

    printf("Gyro offsets: X=%d, Y=%d, Z=%d\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    printf("Accel offsets: X=%d, Y=%d, Z=%d\n", accel_offset[0], accel_offset[1], accel_offset[2]);
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

void compute_angles(int16_t accel[3], int16_t gyro[3], float dt) {
    // Calculate angles from raw accelerometer values
    float accel_angle_x = atan2f((float)accel[1], (float)accel[2]) * 180.0f / M_PI; // Pitch
    float accel_angle_y = atan2f((float)accel[0], (float)accel[2]) * 180.0f / M_PI; // Roll

    // Gyroscope rate in degrees per second
    float gyro_rate_x = gyro[0] / 65.5f; // Assuming 16-bit gyro range of ±2000 dps
    float gyro_rate_y = gyro[1] / 65.5f;
    float gyro_rate_z = gyro[2] / 65.5f;

    // Predict angles from gyroscope data
    float gyro_angle_x = 0.0f;
    float gyro_angle_y = 0.0f;
    float gyro_angle_z = 0.0f;

    gyro_angle_x += gyro_rate_x * dt;
    gyro_angle_y += gyro_rate_y * dt;
    gyro_angle_z += gyro_rate_z * dt;

    // Output results for debugging
    printf("Raw Accel: X=%d, Y=%d, Z=%d\n", accel[0], accel[1], accel[2]);
    printf("Raw Gyro: X=%d, Y=%d, Z=%d\n", gyro[0], gyro[1], gyro[2]);
    printf("Accel Angle X (Pitch): %f\n", accel_angle_x);
    printf("Accel Angle Y (Roll): %f\n", accel_angle_y);
    printf("Gyro Angle X: %f\n", gyro_angle_x);
    printf("Gyro Angle Y: %f\n", gyro_angle_y);
    printf("Gyro Angle Z: %f\n", gyro_angle_z);
}



// Core 1 Task: Sensor Update
void sensor_task() {
    printf("Core 1: Initializing I2C and Sensor Readings...\n");
    /*init_i2c(); ms5611 renable later
    i2c_write_byte(MS5611_ADDR, 0x1E); // Reset sensor
    sleep_ms(10);
    read_calibration_data();
    */
    spi_init(SPI_PORT, 5000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));
    // Configure interrupt pin
    gpio_init(INTERRUPT_PIN);                              // Initialize GPIO for interrupt
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);                  // Set as input
    gpio_pull_up(INTERRUPT_PIN);                           // Enable pull-up resistor
    configure_gpio_interrupt();                            // Attach interrupt and callback
    mpu9250_enable_interrupt();                            // Enable interrupt on MPU9250
    
    // Perform calibration
    calibrate_sensors(gyro_offset, accel_offset);
    //uint32_t start_time, end_time;
    while (1) {
        // Lock the mutex to update the front buffer
        uint32_t prev_time = to_us_since_boot(get_absolute_time());
        uint32_t current_time;

        // Start timing
        //start_time = to_us_since_boot(get_absolute_time());
        
        if (data_ready) { //change this to front buffers
            //printf("Interrupt triggered. Reading data...\n");
            mpu9250_read_raw(back_buffer.accel, back_buffer.gyro, &back_buffer.temp); // Read data from MPU9250
            current_time = to_us_since_boot(get_absolute_time());
            back_buffer.dt = (current_time - prev_time) / 1000000.0f;  // Convert to seconds
            prev_time = current_time;
            data_ready = false;                  // Reset the flag after reading

            // Apply offsets
            for (int i = 0; i < 3; i++) {
                back_buffer.gyro[i] -= gyro_offset[i];
                back_buffer.accel[i] -= accel_offset[i];
            }

        }
        //printf("Core 1: Mutex acquired\n");
        mutex_enter_blocking(&buffer_mutex);
        memcpy(&front_buffer, &back_buffer, sizeof(SensorBuffer));
        mutex_exit(&buffer_mutex);

        /*// Read raw pressure
        write_raw_value(0x40);
        sleep_ms(3); // Allow conversion to complete
        back_buffer.D1 = read_raw_value(0x40);

        // Read raw temperature
        write_raw_value(0x50);
        sleep_ms(3); // Allow conversion to complete
        back_buffer.D2 = read_raw_value(0x50);

        // Signal that new data is ready
        new_data_ready = true;
        */
        // Unlock the mutex

        //printf("Core 1: Mutex released\n");
        //end_time = to_us_since_boot(get_absolute_time()); //usually around 200 us
        //printf("Core 1 loop execution time: %u µs\n", end_time - start_time);
        // Sleep before the next reading
        sleep_us(500); // Adjust delay as needed
    }
}



int main() {
    stdio_init_all();


    pwm_init_motor(PWM_PIN_1);
    pwm_init_motor(PWM_PIN_2);
    pwm_init_motor(PWM_PIN_3);
    pwm_init_motor(PWM_PIN_4);

    calibrate_escs();

    /*i2c_init_magnetometer();
    mpu9250_reset();
    mpu9250_enable_magnetometer(); */
    SensorBuffer local_buffer;
    // Initialize the mutex
    mutex_init(&buffer_mutex);

    // Launch the sensor task on Core 1
    multicore_launch_core1(sensor_task);
    
    // Initialize altitude and angle Kalman filters

    //AltitudeKalmanFilter altitude_kf;
    //altitude_kalman_init(&altitude_kf);
    kalman_init(&kalmanX);
    kalman_init(&kalmanY);
    kalman_init(&kalmanZ);

    // Initialize PID controllers
    PID pid_altitude, pid_pitch, pid_roll, pid_yaw; 
    //pid_init(&pid_altitude, 1.0f, 0.1f, 0.05f); // Adjust Kp, Ki, Kd as needed
    pid_init(&pid_pitch, 1.5f, 0.1f, 0.2f);
    pid_init(&pid_roll, 1.5f, 0.1f, 0.2f);
    pid_init(&pid_yaw, 1.0f, 0.0f, 0.1f);





    // Initialize control variables
    float angles[3]; // Filtered angles: [pitch, roll, yaw]
    //float desired_velocity = 0.0f; // Target vertical velocity (m/s)
    float base_throttle = 1500.0f; // Base throttle for hover (µs)
    //const float decay_rate = 0.1f;  // Adjust how fast the velocity returns to 0
    //const float epsilon = 0.01f;    // Small threshold to stop adjustments
    uint16_t duty1, duty2, duty3, duty4;
    uint32_t current_time, pressure_adc, elapsed_time;
    float dt, measured_altitude, altitude_output, pitch_output, roll_output, yaw_output, motor1, motor2, motor3, motor4;
    int ch, loop_counter = 0;
    uint32_t message_timer = to_ms_since_boot(get_absolute_time());

    //uint32_t last_altitude_update_time = 0; // Time of the last MS5611 update
    //float altitude_dt = 0.0f;              // Delta time for the Kalman filter
    sleep_ms(100);
    uint32_t prev_time = to_us_since_boot(get_absolute_time());
    absolute_time_t loop_start = prev_time;
    while (1) {
    uint32_t start_time, section_start, section_end, end_time;

    start_time = to_us_since_boot(get_absolute_time()); // Start of loop
    section_start = start_time;

    // Timing for dt calculation
    /*current_time = to_us_since_boot(get_absolute_time());
    dt = (current_time - prev_time) / 1000000.0f; // Convert microseconds to seconds
    prev_time = current_time;*/
    /*    if (dt <= 0 || dt > 1.0f) {
        continue; // Skip iteration if dt is invalid
    }
    
    if (dt <= 0 || dt > 1.0f) {
        printf("Error: Invalid dt value: %f\n", dt);
        prev_time = current_time; // Avoid further invalid dt
        continue;
    }
    prev_time = current_time; */


    //printf("Time for dt calculation: %u µs\n", section_end - section_start);


    // MS5611 sensor updates
    // Check if new data is ready
    /*if (new_data_ready) {
            // Lock the mutex to swap buffers
            mutex_enter_blocking(&buffer_mutex);

                // Swap front and back buffers
            front_buffer = back_buffer;

                // Clear the new data flag
            new_data_ready = false;

                // Unlock the mutex
            mutex_exit(&buffer_mutex);

                // Process the updated data
            printf("Core 0: D1 = %u, D2 = %u\n", front_buffer.D1, front_buffer.D2);
        }*/
    //loop_counter = (loop_counter + 1) % 2; // Wrap-around counter for MS5611

    //printf("Time for MS5611 updates: %u µs\n", section_end - section_start);




    //printf("Time for MPU9250 read and offset application: %u µs\n", section_end - section_start);


    
    mutex_enter_blocking(&buffer_mutex);
    memcpy(&local_buffer, &front_buffer, sizeof(SensorBuffer));
    mutex_exit(&buffer_mutex);
    // Kalman filter updates
    kalman_filter_update(local_buffer.accel, local_buffer.gyro, local_buffer.dt, angles);


    /*printf("Raw Gyro: X=%d, Y=%d, Z=%d\n", local_buffer.gyro[0], local_buffer.gyro[1], local_buffer.gyro[2]);
    printf("Raw Accel: X=%d, Y=%d, Z=%d\n", local_buffer.accel[0], local_buffer.accel[1], local_buffer.accel[2]);
    printf("kalmanAngle X: %f\n", kalmanX.angle);
    printf("kalman Bias X: %f\n", kalmanX.bias); */



    //printf("Time for Kalman filter updates: %u µs\n", section_end - section_start);

        // User input to control base throttle
    int ch = getchar_timeout_us(0); // Non-blocking input
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '+') {
                base_throttle = fminf(base_throttle + 10, 2000.0f); // Increase throttle
                //printf("Base throttle increased to: %f\n", base_throttle);
            } else if (ch == '-') {
                base_throttle = fmaxf(base_throttle - 10, 1000.0f); // Decrease throttle
                //printf("Base throttle decreased to: %f\n", base_throttle);
            }
        }


    // User input processing
    /*ch = getchar_timeout_us(0); // Non-blocking input
    if (ch != PICO_ERROR_TIMEOUT) {
        if (ch == ' ') {
            desired_velocity += 0.1f; // Increase vertical velocity
        } else if (ch == 'z') {
            desired_velocity -= 0.1f; // Decrease vertical velocity
        }
    } else {
        // Decay desired_velocity toward 0 when no input is detected
        if (fabsf(desired_velocity) > epsilon) {
            if (desired_velocity > 0) {
                desired_velocity = fmaxf(0.0f, desired_velocity - decay_rate * dt);
            } else {
                desired_velocity = fminf(0.0f, desired_velocity + decay_rate * dt);
            }
        }
    }
    desired_velocity = fmaxf(-2.0f, fminf(2.0f, desired_velocity));

    section_end = to_us_since_boot(get_absolute_time());*/
    //printf("Time for user input processing: %u µs\n", section_end - section_start);
    // PID and motor speed calculations
    //altitude_output = pid_compute(&pid_altitude, desired_velocity, altitude_kf.velocity, dt);
    pitch_output = pid_compute(&pid_pitch, 0.0f, angles[0], dt); // Target pitch: 0 degrees
    roll_output = pid_compute(&pid_roll, 0.0f, angles[1], dt);  // Target roll: 0 degrees
    yaw_output = pid_compute(&pid_yaw, 0.0f, angles[2], dt);   // Target yaw: 0 degrees


    //insert base throttle basic controls
    //base_throttle = fmaxf(1000, fminf(2000, base_throttle + altitude_output));



    motor1 = base_throttle - pitch_output + roll_output + yaw_output;  // Front Right
    motor2 = base_throttle - pitch_output - roll_output - yaw_output;  // Front Left
    motor3 = base_throttle + pitch_output + roll_output - yaw_output;  // Back Left
    motor4 = base_throttle + pitch_output - roll_output + yaw_output;  // Back Right

    motor1 = fmaxf(1000, fminf(2000, motor1));
    motor2 = fmaxf(1000, fminf(2000, motor2));
    motor3 = fmaxf(1000, fminf(2000, motor3));
    motor4 = fmaxf(1000, fminf(2000, motor4));

    duty1 = (motor1 - 1000) * 125 / 1000;
    duty2 = (motor2 - 1000) * 125 / 1000;
    duty3 = (motor3 - 1000) * 125 / 1000;
    duty4 = (motor4 - 1000) * 125 / 1000;

    pwm_set_gpio_level(PWM_PIN_1, duty1);
    pwm_set_gpio_level(PWM_PIN_2, duty2);
    pwm_set_gpio_level(PWM_PIN_3, duty3);
    pwm_set_gpio_level(PWM_PIN_4, duty4);


    //printf("Time for PID and motor calculations: %u µs\n", section_end - section_start);
        // Print messages once every 500 ms
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - message_timer >= 500) {
            compute_angles(local_buffer.accel, local_buffer.gyro, dt);
            printf("Kalman Angle X: %f\n", kalmanX.angle);
            printf("Kalman Bias X: %f\n", kalmanX.bias);
            printf("Base throttle: %f\n", base_throttle);
            message_timer = now;
        }

        // Ensure 1 kHz loop frequency
        end_time = to_us_since_boot(get_absolute_time());
        uint32_t elapsed_time = end_time - start_time;
        if (elapsed_time < 2000) {
            sleep_us(2000 - elapsed_time);
        } else {
            printf("Warning: Loop overrun detected. Elapsed time: %u µs\n", elapsed_time);
        }
    }
}
