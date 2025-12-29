/*Project 2 with some missing functionality 
Ignition system - does not turn off on second press of ignition switch
Headlight system - implemented with PWM rather than just on/off 
Only checks for one thresthold, and should test for two to remember
whether it is nightish or dayish.
*/

#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "driver/ledc.h"
//Needed for oneshot conversion functions          
#include "esp_adc/adc_oneshot.h"                     
 //Needed for calibrarion function
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
// Macros for Project 2 
#define ADC_WIDTH   ADC_BITWIDTH_12         //Choose 12 bit Analog to Digital Conversion
#define ADC_ATTEN   ADC_ATTEN_DB_12         //divide by 4 so ADC sees 0- .825V
#define ADC_LIGHT    ADC_CHANNEL_0           //ADC1_CH0 for the light input
#define ADC_MODE    ADC_CHANNEL_2           //ADC1_CH2 for the mode input
#define ADC_UNIT    ADC_UNIT_1

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define HEAD_LIGHTS             (10) // Define the output for headlights
#define LEDC_CHANNEL            LEDC_CHANNEL_0 // Used for both headlights
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (500) // Frequency in Hertz. Set frequency at 500 Hz (2ms period)

#define LED_BRITE               (8191)
#define LED_OFF                 (0)
#define LED_DIM                 (4096)

#define NIGHT                   (2048)      //Threshold for the light sensor (would need adjustment)

// Macros for Project 1 I/O
//Switches
#define PO 4
#define PSB 5
#define DO 6
#define DS 7
#define IG 15
//LEDs
#define IG_EN 16 //Green
#define EN_ST 17
//Buzzer
#define BUZZ 18


// Function prototype for adc initialization
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
// Function prototype for ledc initialization
static void ledc_init(void);
// Function prototype for ignition enable I/O initialization
void GPIO_init();
// Function prototype for specifying delays in microseconds
void delay_ms(int t);
//Semaphore to let headlight task know that ignition is on
//SemaphoreHandle_t xBinarySemaphore; //Caused "panic_abort"
bool engine_started = false;  //Try a simple global instead

void headlight_task() 
{
//Headlight task
    int lightValue, modeValue;
    bool auto_mode;
    //Set up one-shot conversion
    adc_oneshot_unit_handle_t adc1_handle;     //Define a handle for oneshot 
    adc_oneshot_unit_init_cfg_t init_config1 = {  //Configure which ADC to use
        .unit_id = ADC_UNIT,
        };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  //apply oneshot initial configuration

    adc_oneshot_chan_cfg_t config = {          //Configure bitwidth and attenutation
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,  
    };
    adc_oneshot_config_channel(adc1_handle, ADC_LIGHT, &config); //apply oneshot configuration for light
    adc_oneshot_config_channel(adc1_handle, ADC_MODE, &config);  //apply oneshot configuration for mode
    //ADC1 Calibration Init and check
    adc_cali_handle_t adc1_cali_chan_handle = NULL;
    bool cal1_ret = adc_calibration_init(ADC_UNIT_1, ADC_LIGHT, ADC_ATTEN, &adc1_cali_chan_handle);
    bool cal2_ret = adc_calibration_init(ADC_UNIT_1, ADC_MODE, ADC_ATTEN, &adc1_cali_chan_handle);
    if (cal1_ret && cal2_ret) { printf("Calibrated \n");} else {printf("Not calibrated \n");}

    // Set the LEDC peripheral configuration
    ledc_init();

    // Wait for ignition started semaphore is true
    //    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)
        while(1) {
            if (engine_started)
        {
            printf("Ignition Started in Ignition Task\n");
            //Read mode potentiometer
            adc_oneshot_read(adc1_handle, ADC_MODE, &modeValue);
            if (modeValue < 1000) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_OFF); //OFF
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);    
                printf("Headlights Off %d \n", modeValue);
            } else {
                if (modeValue < 2500) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_DIM); //ON
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);    
                printf("Headlights ON %d \n", modeValue);  
                }
                else {
                    auto_mode = true;                   //Auto
                    printf("Auto Mode %d \n", modeValue);
                }
                }
  
            if (auto_mode) {
                //Read light sensor (will be light sensor)
                adc_oneshot_read(adc1_handle, ADC_LIGHT, &lightValue);
                if (lightValue < NIGHT) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_BRITE);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);    
                    printf("Night time %d \n", lightValue);
                } else {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LED_OFF);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL); 
                    printf("Day time %d \n", lightValue);   
                }
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS); //delay 10 ms to avoid watchdog timer errors
    }
}

//Ignition enable task
void ignition_enable_task() {
    GPIO_init();

  //Declare some variables
    bool IG_enabled = false;
    bool System_started = false;
    bool Attempted = false;
   
   while (1) {
      //Check for driver in seat to start system
      if (!gpio_get_level(DO)&& !System_started) {
        printf("Welcome to the enhanced car alarm model 218-W26 \n"); //too many times
        System_started = true;
        }
      //Check for ignition enable condition
      if (!gpio_get_level(DO) &&
            !gpio_get_level(PO) &&
            !gpio_get_level(DS) &&
            !gpio_get_level(PSB)) {
                gpio_set_level(IG_EN, 1);  //turn on ignition enable
                IG_enabled = true;
            }
            else {
                gpio_set_level(IG_EN, 0);  //turn off ignition enable
                IG_enabled = false;
            }
      //Check for ignition button pressed
      if (!gpio_get_level(IG)) {
        if (IG_enabled) {
            gpio_set_level(EN_ST, 1); //start engine
            engine_started = true;  //Used to enable headlight system
            gpio_set_level(IG_EN, 0); //turn off ignition enabled
            printf("Engine Started\n");
            Attempted = true;
        }
        else {
            gpio_set_level(BUZZ, 1);
            printf("Ignition inhibited\n");
            if(gpio_get_level(DO)) printf("Driver Seat Unoccupied\n");
            if(gpio_get_level(PO)) printf("Passenger Seat Unoccupied\n");
            if(gpio_get_level(DS)) printf("Driver Unbuckled\n");
            if(gpio_get_level(PSB)) printf("Passenger Unbuckled\n");
            Attempted = true;
        }
      }
      delay_ms(100);  //loop delay
      //Removed code for lockout - need to add code to turn off ignition 
   }

}

void app_main(void)
{  
// Create the headlight task
    xTaskCreate(headlight_task, "Headlight_Task", 2048, NULL, 5, NULL);

// Create the ignition enable task
    xTaskCreate(ignition_enable_task, "Ignition_Enable_Task", 2048, NULL, 5, NULL);
}

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

        // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HEAD_LIGHTS,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    return calibrated;
}

//A function to initialize GPIOs
void GPIO_init() {
//Switches are inputs with internal pullup
    gpio_reset_pin(PO);
     gpio_set_direction(PO, GPIO_MODE_INPUT);
      gpio_pullup_en(PO);
    gpio_reset_pin(PSB);
     gpio_set_direction(PSB, GPIO_MODE_INPUT);
      gpio_pullup_en(PSB);
    gpio_reset_pin(DO);
     gpio_set_direction(DO, GPIO_MODE_INPUT);
      gpio_pullup_en(DO);
    gpio_reset_pin(DS);
     gpio_set_direction(DS, GPIO_MODE_INPUT);
      gpio_pullup_en(DS);
    gpio_reset_pin(IG);
     gpio_set_direction(IG, GPIO_MODE_INPUT);
      gpio_pullup_en(IG);
  // LEDs
    gpio_reset_pin(IG_EN);
     gpio_set_direction(IG_EN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(EN_ST);
     gpio_set_direction(EN_ST, GPIO_MODE_OUTPUT);
  //Buzzer
    gpio_reset_pin(BUZZ);
     gpio_set_direction(BUZZ, GPIO_MODE_OUTPUT);
   
  //Initialize outputs
     gpio_set_level(IG_EN, 0);
     gpio_set_level(EN_ST, 0);
     gpio_set_level(BUZZ, 0);
}
//A function to specify delays in miliseconds
void delay_ms(int t) {
    vTaskDelay(t /portTICK_PERIOD_MS);
}
