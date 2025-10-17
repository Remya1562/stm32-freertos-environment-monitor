
#ifndef BME280_H_
#define BME280_H_

#include"stm32f4xx.h"
#include<stdint.h>
#include<stdio.h>

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData;
typedef struct {
	float temperature;
	float humidity;
} SensorData;

void bme280_init(void);
SensorData bme280_read(void);




#endif /* BME280_H_ */
