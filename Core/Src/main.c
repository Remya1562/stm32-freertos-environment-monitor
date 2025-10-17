#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "usart2.h"
#include "gpio.h"
#include "bme280.h"
#include "adc.h"
#include "i2c.h"
#include "timer.h"

// --------------------------- Data Structures ---------------------------

// Struct for inter-task communication (LiveStream → RGB)
typedef struct {
    float measuredTemp;     // Current temperature from BME280
    float setThreshold;     // Adjustable temperature limit
} RGBData;

// --------------------------- Global Variables --------------------------

SemaphoreHandle_t xButtonSemaphore;
QueueHandle_t xRGBQueue;

typedef enum {
    live_streaming,
    threshold_adjust
} SystemMode;

volatile SystemMode current_mode = live_streaming;
volatile float tempThreshold = 40.0f;
volatile float current_temp = 25.0f;

// --------------------------- Function Prototypes -----------------------

void update_rgb_color(float temp, float threshold);

// --------------------------- Functions -------------------------------

// RGB LED update logic (common anode)
void update_rgb_color(float temp, float threshold) {
    if (current_mode == live_streaming) {
        if (temp < 18.0f)
            set_rgb(1000, 1000, 0);   // Blue
        else if (temp <= threshold)
            set_rgb(1000, 0, 1000);   // Green
        else
            set_rgb(0, 1000, 1000);   // Red
    } else {
        // Freeze LED color during threshold adjustment
    }
}

// --------------------------- RTOS Tasks -------------------------------

// Task: Read sensor data or adjust threshold
void vTaskLiveStreaming(void *pvParameters) {
    RGBData rgbData;

    while (1) {
        if (current_mode == live_streaming) {
            SensorData d = bme280_read();
            current_temp = d.temperature;

            rgbData.measuredTemp = d.temperature;
            rgbData.setThreshold = tempThreshold;

            // Send temperature + threshold info to RGB task
            xQueueSend(xRGBQueue, &rgbData, 0);

            printf("Temp = %.2f C, Hum = %.2f %%, Threshold = %.2f°C\r\n",
                   d.temperature, d.humidity, tempThreshold);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (current_mode == threshold_adjust) {
            uint32_t adc_val = adc1_read();
            tempThreshold = 25.0f + ((float)adc_val / 4095.0f) * 20.0f;

            printf("Adjusting threshold: %.2f°C\r\n", tempThreshold);

            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

// Task: Update LED based on latest temperature data from queue
void vTaskRGBLed(void *pvParameters) {
    RGBData rgbData;

    while (1) {
        if (xQueueReceive(xRGBQueue, &rgbData, portMAX_DELAY) == pdTRUE) {
            update_rgb_color(rgbData.measuredTemp, rgbData.setThreshold);
        }
    }
}

// Task: Handle mode switching via button press
void vTaskModeManager(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdTRUE) {
            if (current_mode == live_streaming) {
                current_mode = threshold_adjust;
                printf("Mode switched: THRESHOLD_ADJUST\r\n");
            } else {
                current_mode = live_streaming;
                printf("Mode switched: LIVE_STREAMING\r\n");
            }
        }
    }
}

// --------------------------- Main Function ----------------------------

int main(void) {
    uart_tx_init();
    i2c1_init();
    bme280_init();
    adc1_init();
    button_exti_init();
    tim2_debounce_init();
    timer_init();

    printf("System Initialized\r\n");

    // Create RTOS synchronization primitives
    xButtonSemaphore = xSemaphoreCreateBinary();
    xRGBQueue = xQueueCreate(5, sizeof(RGBData));

    if (xButtonSemaphore == NULL || xRGBQueue == NULL) {
        printf("Failed to create semaphore or queue!\r\n");
        while (1);
    }

    // Create RTOS tasks
    xTaskCreate(vTaskLiveStreaming, "LiveStream", 512, NULL, 2, NULL);
    xTaskCreate(vTaskRGBLed, "RGBControl", 256, NULL, 1, NULL);
    xTaskCreate(vTaskModeManager, "ModeManager", 256, NULL, 3, NULL);

    // Configure NVIC priorities for interrupts
    NVIC_SetPriority(EXTI1_IRQn, 5);
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Start scheduler
    vTaskStartScheduler();

    while (1);
}

// --------------------------- Interrupt Handlers ------------------------

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U << 1)) {
        EXTI->PR |= (1U << 1);
        EXTI->IMR &= ~(1U << 1);
        TIM2->SR &= ~(1U << 0);
        TIM2->CNT = 0;
        TIM2->CR1 |= (1U << 0);
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & 1) {
        TIM2->SR &= ~1;
        TIM2->CR1 &= ~(1U << 0);

        if (GPIOA->IDR & (1U << 1)) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        EXTI->IMR |= (1U << 1);
    }
}
//#include "stm32f4xx.h"
//#include <stdio.h>
//#include <stdint.h>
//
//#include "FreeRTOS.h"
//#include "task.h"
//#include "semphr.h"
//
//#include "usart2.h"
//#include "gpio.h"
//#include "bme280.h"
//#include "adc.h"
//#include "i2c.h"
//#include"timer.h"
//
//SemaphoreHandle_t xButtonSemaphore;
//
//typedef enum{
//		live_streaming,
//		threshold_adjust
//	}SystemMode;
//
//volatile SystemMode current_mode = live_streaming;
//volatile float tempThreshold = 40.0f;
//volatile float current_temp = 25.0f;
//
//void update_rgb_color(float temp, float threshold) {
//    if (current_mode == live_streaming) {
//        if (temp < 18.0f)
//            set_rgb(1000, 1000, 0);   // Blue
//        else if (temp <= threshold)
//            set_rgb(1000, 0, 1000);   // Green
//        else
//            set_rgb(0, 1000, 1000);   // Red
//    }
//    else {
//    }
//}
//
//
//
//void vTaskRGBLed(void *pvParameters) {
//    while(1) {
//        update_rgb_color(current_temp, tempThreshold);
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//}
//
//void vTaskLiveStreaming(void *pvParameters){
//	while(1){
//		if(current_mode==live_streaming){
//			SensorData d=bme280_read();
//			current_temp = d.temperature;
//			printf("Temp = %.2f C, Hum = %.2f %%, Threshold = %.2f°C\r\n",d.temperature, d.humidity, tempThreshold);
//			vTaskDelay(pdMS_TO_TICKS(1000));
//		}
//		else if(current_mode == threshold_adjust) {
//			 uint32_t adc_val = adc1_read();
//		     tempThreshold = 25.0f + ((float)adc_val / 4095.0f) * 20.0f;
//		     printf("Adjusting threshold: %.2f°C\r\n", tempThreshold);
//		     vTaskDelay(pdMS_TO_TICKS(500));
//		}
//	}
//}
//
//
//void vTaskModeManager(void *pvParameters){
//    while (1){
//		if(xSemaphoreTake(xButtonSemaphore,portMAX_DELAY)==pdTRUE){
//			if (current_mode == live_streaming) {
//			    current_mode = threshold_adjust;
//			    printf("Mode switched:THRESHOLD_ADJUST\r\n");
//			}
//			else{
//				current_mode = live_streaming;
//				printf("Mode switched: LIVE_STREAMING\r\n");
//			}
//		}
//
//	}
//}
//
//int main(void){
//	uart_tx_init();
//	i2c1_init();
//	bme280_init();
//	adc1_init();
//	button_exti_init();
//	tim2_debounce_init();
//	timer_init();
//
//	printf("System Initialized\r\n");
//
//    xButtonSemaphore = xSemaphoreCreateBinary();
//    printf("Semaphore created: %p\n", xButtonSemaphore);
//    if (xButtonSemaphore == NULL) {
//           printf("Failed to create button semaphore!\r\n");
//           while(1);
//       }
//
//    xTaskCreate(vTaskLiveStreaming,"LiveStream",512,NULL,1,NULL);
//    xTaskCreate(vTaskModeManager,"ModeManager",256,NULL,2,NULL);
//    xTaskCreate(vTaskRGBLed,"RGBControl",256,NULL,1,NULL);
//
//    NVIC_SetPriority(EXTI1_IRQn, 5);
//    NVIC_SetPriority(TIM2_IRQn, 5);
//
//    NVIC_EnableIRQ(EXTI1_IRQn);
//    NVIC_EnableIRQ(TIM2_IRQn);
//
//    vTaskStartScheduler();
//
//    while (1);
//
//}
//void EXTI1_IRQHandler(void) {
//    if(EXTI->PR & (1U << 1)) {
//        EXTI->PR |= (1U << 1);
//        EXTI->IMR &= ~(1U << 1);
//        TIM2->SR &= ~(1U << 0);
//        TIM2->CNT = 0;
//        TIM2->CR1 |= (1U << 0);
//    }
//}
//void TIM2_IRQHandler(void) {
//    if(TIM2->SR & 1) {
//        TIM2->SR &= ~1;
//        TIM2->CR1 &= ~(1U << 0);
//        if(GPIOA->IDR & (1U << 1)) {
//            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//            xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }
//
//        EXTI->IMR |= (1U << 1);
//    }
//}
