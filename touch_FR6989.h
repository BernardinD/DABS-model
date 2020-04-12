/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef TOUCH_FR6989_H_
#define TOUCH_FR6989_H_

/*
 * X- port definitions.
 */
#define TOUCH_X_MINUS_PORT      GPIO_PORT_P2
#define TOUCH_X_MINUS_PIN       GPIO_PIN3

/*
 * Y- port definitions.
 */
#define TOUCH_Y_MINUS_PORT      GPIO_PORT_P4
#define TOUCH_Y_MINUS_PIN       GPIO_PIN7

/*
 * SW1 pin definitions.
 */
#define TOUCH_SW1_PORT          GPIO_PORT_P1
#define TOUCH_SW1_PIN           GPIO_PIN1

/*
 * X+ port and ADC12 definitions.
 */
#define TOUCH_X_PLUS_PORT       /*GPIO_PORT_P9*/GPIO_PORT_P8
#define TOUCH_X_PLUS_PIN        /*GPIO_PIN7*/GPIO_PIN5
#define TOUCH_X_PLUS_INPUT      ADC12_B_INPUT_A7
#define TOUCH_X_PLUS_MEMORY     ADC12_B_MEMORY_1
#define TOUCH_X_PLUS_IFG        ADC12IFG1

/*
 * Y+ port and ADC12 definitions.
 */
#define TOUCH_Y_PLUS_PORT       /*GPIO_PORT_P9*/GPIO_PORT_P8
#define TOUCH_Y_PLUS_PIN        /*GPIO_PIN6*/GPIO_PIN4
#define TOUCH_Y_PLUS_INPUT      ADC12_B_INPUT_A6
#define TOUCH_Y_PLUS_MEMORY     ADC12_B_MEMORY_0
#define TOUCH_Y_PLUS_IFG        ADC12IFG0

/*
 * Threshold for detecting if there is a touch, specific to ADC resolution.
 */
#define TOUCH_THRESHOLD         350

/*
 * Touch screen calibration value.
 */
#define TOUCH_X_MINIMUM (750 << 2)//625
#define TOUCH_X_MAXIMUM (3500 << 2)//3000
#define TOUCH_Y_MINIMUM (1130 << 2)//1130
#define TOUCH_Y_MAXIMUM (3400 << 2)//3400

#define TOUCH_OVERSAMPLE                        8
#define TOUCH_AVERAGE_DIVISOR           3

#define TOUCH_CALIBRATION_KEY       0xA5A5A5A5

#include <stdbool.h>
#include <stdint.h>

typedef struct touch_calibration
{
    uint16_t xMin;
    uint16_t xMax;
    uint16_t yMin;
    uint16_t yMax;
    uint32_t key;
} touch_calibration;

/*
 * Typedef for touch context structure.
 */
typedef struct touch_context
{
    bool touch;
    uint16_t x;
    uint16_t y;
    touch_calibration touch_calibrationData;
} touch_context;
/*
 * Radius of calibration circles.
 */
#define TOUCH_CALIBRATION_RADIUS    16
/*
 * Globals
 */
extern touch_context touch_currentTouch;

/*
 * Function prototypes.
 */
void touch_initInterface(void);
void touch_updateCurrentTouch(touch_context *data);
void touch_calibrate(void);
void touch_detectedTouch(void);
bool cal_touch_detectedTouch(void);
uint16_t scaleX(uint16_t);
uint16_t scaleY(uint16_t);

#endif //TOUCH_H_
