#include "Foc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "Application.h"

const char *TAG = "main";

extern "C" void app_main(void)
{
    Application app;
    app.init();
    app.run();
}