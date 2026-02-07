#include "pico/stdlib.h"
#include "drivers.h"
#include "utils.h"

int main(void) {
    stdio_init_all();

    const uint LED = PICO_DEFAULT_LED_PIN;
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    while (true) {
        gpio_put(LED, 1);
        sleep_ms(250);
        gpio_put(LED, 0);
        sleep_ms(250);
    }
}
