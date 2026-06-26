#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inference_engine.h"

void app_main(void)
{
    inference_engine_init();
}