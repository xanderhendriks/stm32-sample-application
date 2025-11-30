//! @file application.c

// Interface for this file
#include "application.h"

// Standard library includes
#include <stdbool.h>
#include <stdio.h>

// Library includes
#include "cmsis_os.h"

// Application includes
#include "main.h"
#include "sensor.h"
#include "version.h"

extern UART_HandleTypeDef huart2;

void Application_RunDefaultTask(void)
{
    int32_t calculate = 0;

    printf("image_id: %d, version: %d.%d.%d-%s\n", (int) IMAGE_ID, (int) VERSION_MAJOR, (int) VERSION_MINOR,
           (int) VERSION_BUGFIX, SHORT_GIT_HASH_STRING);

    calculate = Sensor_GetValue() + 20;

    printf("Calculated value: %ld\n", calculate);

    for (;;)
    {
        uint8_t data[1];

        if (HAL_UART_Receive(&huart2, (uint8_t *) data, 1, 0) == HAL_OK)
        {
            switch (data[0])
            {
                case 'v':
                    printf("image_id: %d, version: %d.%d.%d-%s\n", (int) IMAGE_ID, (int) VERSION_MAJOR,
                           (int) VERSION_MINOR, (int) VERSION_BUGFIX, SHORT_GIT_HASH_STRING);
                    break;

                default:
                case 's':
                    printf("sensor: %ld\n", Sensor_GetValue());
                    break;
            }
        }

        osDelay(10);
    }
}

void Application_RunLedTask(void)
{
    for (;;)
    {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        osDelay(500);
    }
}
