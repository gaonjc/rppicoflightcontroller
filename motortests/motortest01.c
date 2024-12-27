#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define MIN_DUTY_CYCLE 10 // Example: 10%
#define MAX_DUTY_CYCLE 90 // Example: 90%
// Define the PWM GPIO pins and initial PWM settings
#define PWM_PIN_1 15         // GPIO pin for PWM Motor 1
#define PWM_PIN_2 16         // GPIO pin for PWM Motor 2
#define PWM_PIN_3 17         // GPIO pin for PWM Motor 3
#define PWM_PIN_4 18         // GPIO pin for PWM Motor 4
#define PWM_FREQ 50          // Frequency in Hz for ESC (typically 50 Hz for RC ESCs)
#define PWM_WRAP 20000       // For a 20ms period, adjust if needed

uint16_t duty_cycle = 0;  // Start at 1.5ms pulse width (neutral for ESCs)
const uint16_t min_duty = 1000;  // Minimum pulse width (ESC's lowest throttle)
const uint16_t max_duty = 2000;  // Maximum pulse width (ESC's full throttle)

// Function to set the same duty cycle across multiple motors
void set_pwm_duty(uint slice_num, uint16_t duty) {
    pwm_set_gpio_level(PWM_PIN_1, duty);
    pwm_set_gpio_level(PWM_PIN_2, duty);
    pwm_set_gpio_level(PWM_PIN_3, duty);
    pwm_set_gpio_level(PWM_PIN_4, duty);
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

int main() {
    stdio_init_all();
    sleep_ms(10*500);
    // Initialize all GPIO pins for PWM function
    gpio_set_function(PWM_PIN_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_4, GPIO_FUNC_PWM);
    calibrate_escs();
    // Get PWM slice numbers for each motor pin (assumes each pin is on its own slice)
    uint slice_num_1 = pwm_gpio_to_slice_num(PWM_PIN_1);
    uint slice_num_2 = pwm_gpio_to_slice_num(PWM_PIN_2);
    uint slice_num_3 = pwm_gpio_to_slice_num(PWM_PIN_3);
    uint slice_num_4 = pwm_gpio_to_slice_num(PWM_PIN_4);


    // Set wrap and clock divider for each slice to achieve 50 Hz
    pwm_set_wrap(slice_num_1, PWM_WRAP);
    pwm_set_wrap(slice_num_2, PWM_WRAP);
    pwm_set_wrap(slice_num_3, PWM_WRAP);
    pwm_set_wrap(slice_num_4, PWM_WRAP);
    pwm_set_clkdiv(slice_num_1, 125.0f);
    pwm_set_clkdiv(slice_num_2, 125.0f);
    pwm_set_clkdiv(slice_num_3, 125.0f);
    pwm_set_clkdiv(slice_num_4, 125.0f);

    // Set initial duty cycle and enable PWM for each slice
    set_pwm_duty(slice_num_1, duty_cycle);
    set_pwm_duty(slice_num_2, duty_cycle);
    set_pwm_duty(slice_num_3, duty_cycle);
    set_pwm_duty(slice_num_4, duty_cycle);

    pwm_set_enabled(slice_num_1, true);
    pwm_set_enabled(slice_num_2, true);
    pwm_set_enabled(slice_num_3, true);
    pwm_set_enabled(slice_num_4, true);

    printf("ESC control program started. Use space to increase and esc to decrease.\n");
    int duty_cycle1;
    // Main loop for controlling the duty cycle
    while (true) {
        char ch = getchar_timeout_us(0);
        switch (ch) {
        case '1': duty_cycle = MIN_DUTY_CYCLE; break;
        case '2': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 10 / 8 +1000; break;
        case '3': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 20 / 8+1000; break;
        case '4': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 30 / 8+1000; break;
        case '5': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 40 / 8+1000; break;
        case '6': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 50 / 8+1000; break;
        case '7': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 60 / 8+1000; break;
        case '8': duty_cycle = MIN_DUTY_CYCLE + (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * 70 / 8+1000; break;
        case '9': duty_cycle = MAX_DUTY_CYCLE; break;
        case ' ': duty_cycle += 10; break;
        case 0x1B: duty_cycle -= 10; break;
        default:
 
        }      

                set_pwm_duty(slice_num_1, duty_cycle);
                set_pwm_duty(slice_num_2, duty_cycle);
                set_pwm_duty(slice_num_3, duty_cycle);
                set_pwm_duty(slice_num_4, duty_cycle);
                printf("Increased PWM duty: %d\n", duty_cycle);
 

        
        sleep_ms(20);  // Small delay to control rate of changes
    }
}
