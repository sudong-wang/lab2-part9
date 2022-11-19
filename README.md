### lab2-part9

### Intro

In general, we basically use 2 pio(`pio0` and `pio1`) and `sm = 0` to deliever color packet to PIO module and to deliever bit&full packet to WS2812. So PY2040 will receive the color info from APDS9960 via PIO I2C and then deliever it to ws2812 pixel module. Then the neopixel will perform color and brightness changes which APDS9960 detected.

### code

We directly added neopixel function to APDS9960.c in the previous section.


            #include <stdio.h>
            #include <stdlib.h>
            #include <string.h>
            #include <stdbool.h>
            #include "pico/stdlib.h"
            #include "pico/binary_info.h"
            #include "string.h"
            #include "APDS9960.h"
            #include "pio_i2c.h"

            #include "hardware/pio.h"
            #include "hardware/clocks.h"
            #include "ws2812.pio.h"


            #define IS_RGBW true
            #define NUM_PIXELS 150
            #define POWER_ENABLE 1
            #define PROXIMITY_ENABLE 1
            #define ALS_ENABLE 1
            #define GESTURE_ENABLE 0


            #define ALS_GAIN 0
            #define ALS_TIME 219

            #define INIT_CONFIG (GESTURE_ENABLE << 6u) | (PROXIMITY_ENABLE << 2u) | (ALS_ENABLE << 1u) | POWER_ENABLE
            #define WS2812_PIN 12
            #define WS2812_POWER_PIN 11

            const int addr = 0x39;
            const int max_read = 250;

            #define PIN_SDA 22
            #define PIN_SCL 23


            PIO pio = pio0;
            uint sm = 0;
            PIO pio_1 = pio1;
            uint sm_1 = 0;
            
            // set Neopixel
            
            void turn_on_NeoPixel_power(){
                const uint led_pin = WS2812_POWER_PIN;
                gpio_init(led_pin);
                gpio_set_dir(led_pin, GPIO_OUT);
                gpio_put(led_pin,1);
            }


            void neopixel_set_rgb(uint32_t rgb) {
                // convert RGB to GRB
                uint32_t grb = ((rgb & 0xFF0000) >> 8) | ((rgb & 0x00FF00) << 8) | (rgb & 0x0000FF);
                pio_sm_put_blocking(pio_1, 0, grb << 8u);
            }



        void read_rgbc(PIO pio, uint sm,int32_t* r, int32_t* g, int32_t* b, int32_t* c) {

            uint8_t buf[2];
            uint8_t reg = 0x94;
            pio_i2c_write_blocking(pio, sm, addr, &reg, 1, true);  // true to keep master control of bus
            pio_i2c_read_blocking(pio, sm, addr, buf, 8, false);  // false - finished with bus

            *c = (buf[1] << 8) | buf[0];
            *r = (buf[3] << 8) | buf[2];
            *g = (buf[5] << 8) | buf[4];
            *b = (buf[7] << 8) | buf[6];
        }


        int main() {
            stdio_init_all();

            uint offset = pio_add_program(pio, &i2c_program);
            i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);
            sleep_ms(5000);
            
            // Initialize apds9960
            
            uint8_t buf[2];

            buf[0] = 0x80;
            buf[1] = INIT_CONFIG;
            pio_i2c_write_blocking(pio, sm, addr, buf, 2, false);

            buf[0] = 0x81;
            buf[1] = ALS_TIME;
            pio_i2c_write_blocking(pio, sm, addr, buf, 2, false);

            stdio_init_all();
            printf("WS2812 Smoke Test, using pin %d", WS2812_PIN);

            // todo get free sm

            uint offset1 = pio_add_program(pio_1, &ws2812_program);
            turn_on_NeoPixel_power();
            ws2812_program_init(pio_1, sm, offset1, WS2812_PIN, 800000, IS_RGBW);

            int x = 0;
            uint32_t pixel_grb = 0x00000000;


            while (true) {

                int32_t r,g,b,c;


                read_rgbc(pio,sm, &r, &g, &b, &c);


                printf("r:%d, g:%d, b:%d, c:%d\n", r, g, b, c);
                neopixel_set_rgb((r << 8u | g | b >> 8u));
                sleep_ms(100);
            }

            }
    
 ### Result:

The neopixel will perform color and brightness changes which APDS9960 detected. The resylt is shown below:

![4ndwq-xdx3h](https://user-images.githubusercontent.com/113209201/202830625-d335e3c6-8fb3-45cc-984f-ce63c719e334.gif)

           

       
        


        

    

