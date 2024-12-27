#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/mutex.h"

// I2C Configuration
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21
#define MS5611_ADDR 0x76

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

// Shared Buffers
typedef struct {
    uint32_t D1, D2;
} SensorBuffer;

volatile SensorBuffer front_buffer; // Read-only for Core 0
volatile SensorBuffer back_buffer;  // Write-only for Core 1

volatile bool new_data_ready = false; // Flag for new data
mutex_t buffer_mutex; // Mutex to control buffer swaps

// Global variables for calibration
uint16_t C1, C2, C3, C4, C5, C6;
int64_t dT, TEMP, OFF, SENS, T2, OFF2, SENS2;   
float pressure, cTemp, fTemp;

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
    int ret = i2c_init(I2C_PORT, 100000);
    if (ret < 0) {
        printf("Core 1: I2C initialization failed with error %d\n", ret);
        while (1); // Halt if I2C initialization fails
    }
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("Core 1: I2C initialized successfully\n");
}

// Function to write a single command to the sensor
void i2c_write_byte(uint8_t address, uint8_t command) {
    int ret = i2c_write_blocking(I2C_PORT, address, &command, 1, false);
    if (ret < 0) {
        printf("Core 1: I2C write failed with error %d\n", ret);
    }
}

// Function to read multiple bytes from the sensor
void i2c_read_bytes(uint8_t address, uint8_t command, uint8_t *buffer, size_t length) {
    int ret = i2c_write_blocking(I2C_PORT, address, &command, 1, true);
    if (ret < 0) {
        printf("Core 1: I2C write before read failed with error %d\n", ret);
        return;
    }
    ret = i2c_read_blocking(I2C_PORT, address, buffer, length, false);
    if (ret < 0) {
        printf("Core 1: I2C read failed with error %d\n", ret);
    }
}

// Function to read calibration data
void read_calibration_data() {
    uint8_t data[2];

    i2c_read_bytes(MS5611_ADDR, 0xA2, data, 2);
    C1 = data[0] << 8 | data[1];
    printf("Core 1: C1 = %u\n", C1);

    i2c_read_bytes(MS5611_ADDR, 0xA4, data, 2);
    C2 = data[0] << 8 | data[1];
    printf("Core 1: C2 = %u\n", C2);

    i2c_read_bytes(MS5611_ADDR, 0xA6, data, 2);
    C3 = data[0] << 8 | data[1];
    printf("Core 1: C3 = %u\n", C3);

    i2c_read_bytes(MS5611_ADDR, 0xA8, data, 2);
    C4 = data[0] << 8 | data[1];
    printf("Core 1: C4 = %u\n", C4);

    i2c_read_bytes(MS5611_ADDR, 0xAA, data, 2);
    C5 = data[0] << 8 | data[1];
    printf("Core 1: C5 = %u\n", C5);

    i2c_read_bytes(MS5611_ADDR, 0xAC, data, 2);
    C6 = data[0] << 8 | data[1];
    printf("Core 1: C6 = %u\n", C6);
}

// Function to read raw pressure or temperature
void write_raw_value(uint8_t command) {
    i2c_write_byte(MS5611_ADDR, command);
}

uint32_t read_raw_value(uint8_t command) {
    uint8_t data[3];
    
    // Issue the conversion command
    int ret = i2c_write_blocking(I2C_PORT, MS5611_ADDR, &command, 1, true);
    if (ret < 0) {
        printf("Core 1: I2C write failed with error %d for command 0x%X\n", ret, command);
        return 0;
    }

    // Wait for conversion to complete (minimum 3ms for OSR=256)
    sleep_ms(3);

    // Issue the ADC read command (0x00)
    command = 0x00;
    ret = i2c_write_blocking(I2C_PORT, MS5611_ADDR, &command, 1, true);
    if (ret < 0) {
        printf("Core 1: I2C write (ADC read) failed with error %d\n", ret);
        return 0;
    }

    // Read the 3-byte result
    ret = i2c_read_blocking(I2C_PORT, MS5611_ADDR, data, 3, false);
    if (ret < 0) {
        printf("Core 1: I2C read failed with error %d\n", ret);
        return 0;
    }

    uint32_t value = (data[0] << 16) | (data[1] << 8) | data[2];
    printf("Core 1: Read raw value: 0x%X\n", value);
    return value;
}

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

// Core 1: Sensor Task
void sensor_task() {
    spi_init(SPI_PORT, 5000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    init_i2c();       // Initialize I2C
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));   

    i2c_init_magnetometer();
    mpu9250_reset();
    mpu9250_enable_magnetometer();
    // Configure interrupt pin
    gpio_init(INTERRUPT_PIN);                              // Initialize GPIO for interrupt
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);                  // Set as input
    gpio_pull_up(INTERRUPT_PIN);                           // Enable pull-up resistor
    configure_gpio_interrupt();                            // Attach interrupt and callback
    mpu9250_enable_interrupt();                            // Enable interrupt on MPU9250

    printf("Resetting sensor...\n");
    i2c_write_byte(MS5611_ADDR, 0x1E); // Reset sensor
    sleep_ms(10);

    printf("Reading calibration data...\n");
    read_calibration_data();

    while (1) {
        /*    if (new_data_ready) {

                // Swap front and back buffers
            front_buffer = back_buffer;

                // Clear the new data flag
            new_data_ready = false;


                // Process the updated data
            printf("Core 0: D1 = %u, D2 = %u\n", front_buffer.D1, front_buffer.D2);
        }
*/
        printf("Reading raw values...\n");
        write_raw_value(0x40);  
        sleep_ms(3);
        back_buffer.D1 = read_raw_value(0x40); // Pressure conversion (OSR = 256)
        write_raw_value(0x50);
        sleep_ms(3);
        back_buffer.D2 = read_raw_value(0x50); // Temperature conversion (OSR = 256)
        // Write D1 and D2 to the FIFO
        if (multicore_fifo_wready()) {
            multicore_fifo_push_blocking(back_buffer.D1);
            multicore_fifo_push_blocking(back_buffer.D2);
        } else {
            printf("Core 1: FIFO full, skipping write\n");
        }


        sleep_ms(200); // Wait for 2 seconds before next reading
    }
}

// Core 0: Main Task
int main() {
    stdio_init_all();
    sleep_ms(5000); // Wait for the terminal to be ready
    multicore_launch_core1(sensor_task);
    printf("Main: Core 1 launch attempted\n");
    mutex_init(&buffer_mutex);
    printf("Main: Mutex initialized\n");

    sleep_ms(5000); // Wait for the terminal to be ready
    printf("Main: Starting program\n");



    while (1) {
        /* Read front buffer values
        if (mutex_try_enter(&buffer_mutex, 0)) {
            printf("Core 0: Acquired mutex. Reading front buffer\n");
            printf("Core 0: D1=%u, D2=%u\n", front_buffer.D1, front_buffer.D2);
            mutex_exit(&buffer_mutex);
        } else {
            printf("Core 0: Mutex locked. Reading old data\n");
            printf("Core 0: D1=%u, D2=%u\n", front_buffer.D1, front_buffer.D2);
        }*/
        if (multicore_fifo_rvalid()) {
            // Read D1 and D2 from FIFO
            front_buffer.D1 = multicore_fifo_pop_blocking();
            front_buffer.D2 = multicore_fifo_pop_blocking();

            // Process or print the data
            printf("Core 0: D1=%u, D2=%u\n", front_buffer.D1, front_buffer.D2);
        } else {
            printf("Core 0: FIFO empty, waiting for new data old: %u, %u\n", front_buffer.D1, front_buffer.D2);
        }
        calculate_temperature_and_pressure();

        printf("Pressure: %.2f mbar\n", pressure);
        //printf("Temperature in Celsius: %.2f C\n", cTemp);
        printf("Temperature in Fahrenheit: %.2f F\n", fTemp);
        sleep_ms(50); // Simulate data processing delay
    }

    return 0;
}
