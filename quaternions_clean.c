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

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0
#define READ_BIT 0x80

#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_FIFO_EN 0x23
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75

//#define INTERRUPT_PIN 22
//volatile bool data_ready = false; // Flag to indicate data availability

#define QUAT_FIFO_SIZE 16  // 6-Axis Quaternion size

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
    uint8_t buf[] = {0x6B, 0x00}; // PWR_MGMT_1: Reset device
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
    sleep_ms(100);
}


static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_us(1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    sleep_us(1);
}
void check_mpu9250() {
    uint8_t who_am_i;
    read_registers(MPU9250_WHO_AM_I, &who_am_i, 1);
    printf("MPU9250 WHO_AM_I: 0x%02X\n", who_am_i);
}
void mpu9250_enable_dmp() {
    uint8_t buffer[2];

    // Wake up the MPU9250
    buffer[0] = 0x6B; // PWR_MGMT_1 register
    buffer[1] = 0x00;
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

void mpu9250_wake_up() {
    uint8_t buffer[2];

    // Set clock source and wake up device
    buffer[0] = MPU9250_PWR_MGMT_1;
    buffer[1] = 0x01;  // Auto selects best clock
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(100);
}


void mpu9250_configure_quaternion_output() {
    uint8_t buffer[2];

    // Reset FIFO
    buffer[0] = MPU9250_USER_CTRL;
    buffer[1] = 0x44;
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(10);

    // Enable DMP & FIFO
    buffer[0] = MPU9250_USER_CTRL;
    buffer[1] = 0xC0;
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(10);

    // Enable quaternion data in FIFO
    buffer[0] = MPU9250_FIFO_EN;
    buffer[1] = 0xF0;  // Enable quaternion data
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(10);
}


int mpu_read_fifo(uint8_t *buffer, uint16_t length) {
    uint8_t reg = MPU9250_FIFO_R_W | READ_BIT;

    // Ensure FIFO has data
    uint8_t fifo_count[2];
    read_registers(MPU9250_FIFO_COUNTH, fifo_count, 2);
    uint16_t count = (fifo_count[0] << 8) | fifo_count[1];

    if (count < length) {
        printf("FIFO too small, skipping read\n");
        return -1; // FIFO does not have enough data
    }

    // Read FIFO data
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0x00, buffer, length);
    cs_deselect();

    return 0;
}
void mpu9250_reset_fifo() {
    uint8_t buffer[2];

    buffer[0] = MPU9250_USER_CTRL;
    buffer[1] = 0x04;  // Reset FIFO
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect();
    sleep_ms(10);
}

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

// Function to read quaternion values from the MPU9250 DMP FIFO
int read_dmp_quaternion(Quaternion *q) {
    uint8_t buffer[QUAT_FIFO_SIZE];
    int32_t qw, qx, qy, qz;
    int result;

    // Read FIFO count
    uint8_t fifo_count[2];
    read_registers(MPU9250_FIFO_COUNTH, fifo_count, 2);
    uint16_t count = (fifo_count[0] << 8) | fifo_count[1];

    printf("RAW FIFO Count: %d (H: 0x%02X, L: 0x%02X)\n", count, fifo_count[0], fifo_count[1]);

    if (count < QUAT_FIFO_SIZE) {
        printf("FIFO not ready, count: %d\n", count);
        return -1;  // No new data available
    }

    // Read 16 bytes from FIFO
    result = mpu_read_fifo(buffer, QUAT_FIFO_SIZE);
    if (result != 0) {
        printf("Error reading quaternion data\n");
        return result;
    }

    // Convert raw data to 32-bit signed values
    qw = (int32_t)((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3]);
    qx = (int32_t)((buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7]);
    qy = (int32_t)((buffer[8] << 24) | (buffer[9] << 16) | (buffer[10] << 8) | buffer[11]);
    qz = (int32_t)((buffer[12] << 24) | (buffer[13] << 16) | (buffer[14] << 8) | buffer[15]);

    // Convert to floating-point (scaled by 2^30)
    const float scale = 1.0f / (1 << 30);
    q->w = qw * scale;
    q->x = qx * scale;
    q->y = qy * scale;
    q->z = qz * scale;

    return 0;
}

void debug_mpu9250_registers() {
    uint8_t reg_values[10];
    read_registers(MPU9250_PWR_MGMT_1, reg_values, 10);
    printf("PWR_MGMT_1: 0x%02X\n", reg_values[0]);
    printf("PWR_MGMT_2: 0x%02X\n", reg_values[1]);
    printf("USER_CTRL: 0x%02X\n", reg_values[2]);
    printf("FIFO_EN: 0x%02X\n", reg_values[3]);
    printf("INT_ENABLE: 0x%02X\n", reg_values[4]);
    printf("INT_STATUS: 0x%02X\n", reg_values[5]);
    printf("FIFO_COUNTH: 0x%02X\n", reg_values[6]);
    printf("FIFO_COUNTL: 0x%02X\n", reg_values[7]);
}


int main() {
    stdio_init_all();
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Deselect CS initially

    sleep_ms(1000); // Wait for stabilization

    check_mpu9250(); // Verify WHO_AM_I
    mpu9250_reset(); // Reset MPU9250
    mpu9250_wake_up(); // Wake up MPU9250
    mpu9250_enable_dmp(); // Enable DMP
    mpu9250_configure_quaternion_output(); // Configure FIFO for quaternion data

    debug_mpu9250_registers(); // Debug register states

    Quaternion q;
    while (1) {
        uint8_t fifo_count[2];
        read_registers(MPU9250_FIFO_COUNTH, fifo_count, 2);
        uint16_t count = (fifo_count[0] << 8) | fifo_count[1];
        printf("FIFO Count: %d\n", count);

        if (count >= QUAT_FIFO_SIZE) {
            if (read_dmp_quaternion(&q) == 0) {
                printf("Quaternion: w=%.6f, x=%.6f, y=%.6f, z=%.6f\n", q.w, q.x, q.y, q.z);
            }
        } else {
            printf("FIFO empty, retrying...\n");
        }

        sleep_ms(100);
    }
}