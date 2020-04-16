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
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <msp430fr6989.h>
#include <touch_FR6989.h>
#include "driverlib.h"
#include "grlib.h"
#include "kitronix320x240x16_ssd2119_spi.h"

/*
 * Graphic library context
 */
extern Graphics_Context g_sContext;

//static bool cal_touch_detectedTouch(void);

#if defined(__TI_COMPILER_VERSION__)
#pragma LOCATION(touch_calibrationData, 0x4400);
#pragma NOINT(touch_calibrationData);
static touch_calibration touch_calibrationData;
#pragma RETAIN(touch_calibrationData);
/*#elif defined(__IAR_SYSTEMS_ICC__)
#pragma location=0x4400
__no_init static touch_calibration touch_calibrationData;
#else
#error "Compiler not supported!"
//*/
#endif

void Initialize_touch(){

    // Ensure that first configuration does not trigger a toggle
    P2DIR |= TOUCH_X_MINUS_PIN;
    P2OUT |= TOUCH_X_MINUS_PIN;
    // Set interrupt to flag when 'TOUCH_X_MINUS_PIN' goes to 0
    P2IES |= TOUCH_X_MINUS_PIN;    // => Falling edge
    P2IFG &= ~TOUCH_X_MINUS_PIN;

    // Enable interrupts for X-
    P2IE |= TOUCH_X_MINUS_PIN;

    /* Calibrate the touch screen. */
    //touch_calibrationData.key = 0;
    if (touch_calibrationData.key != TOUCH_CALIBRATION_KEY || 0)
        touch_calibrate();


}
void Initialize_ADC() {
    // Divert the pins to analog functionality
    // X-Plus touch: A10/P8.5, for A6 (P8DIR=x, P9SEL1=1, P9SEL0=1)
    //P8SEL0 |= BIT5;
    //P8SEL1 |= BIT5;
    // Y-Plus touch: A10/P8.4, for A7 (P8DIR=x, P9SEL1=1, P9SEL0=1)
    //P8SEL0 |= BIT4;
    //P8SEL1 |= BIT4;

    //P8REN |= BIT5;
    //P8REN |= BIT4;

    // Turn on the ADC module
    ADC12CTL0 |= ADC12ON;

    // Turn off ENC (Enable Conversion) bit while modifying the configuration
    ADC12CTL0 &= ~ADC12ENC;

    //*************** ADC12CTL0 ***************
    // Set ADC12SHT0 (select the number of cycles that you determined)
    // Set the bit ADC12MSC (Multiple Sample and Conversion)
    ADC12CTL0 |= (ADC12SHT00);
    ADC12CTL0 |= (ADC12MSC);

    //*************** ADC12CTL1 ***************
    // Set ADC12SHS (select ADC12SC bit as the trigger)
    // Set ADC12SHP bit
    // Set ADC12DIV (select the divider you determined)
    // Set ADC12SSEL (select MODOSC)
    // Set ADC12CONSEQ (select sequence-of-channels)
    //ADC12CTL1 &= ~(ADC12SHS2|ADC12SHS1|ADC12SHS0|ADC12SSEL1|ADC12SSEL0);
    ADC12CTL1 |= (ADC12SHP);
    ADC12CTL1 |= (ADC12DIV1|ADC12DIV0);
    ADC12CTL1 |= (ADC12CONSEQ_1);

    //*************** ADC12CTL2 ***************
    // Set ADC12RES (select 12-bit resolution)
    // Set ADC12DF (select unsigned binary format)
    ADC12CTL2 |= ADC12RES_2;

    //*************** ADC12CTL3 ***************
    // Leave all fields at default values
    //ADC12CTL3 |= ADC12CSTARTADD_0;

    //*************** ADC12MCTL0 ***************
    // Set ADC12VRSEL (select VR+=AVCC, VR-=AVSS)
    // Set ADC12INCH (select the analog channel that you found)
    // Set ADC12EOS (last conversion in ADC12MEM1)

    setTouchADC();

    // Turn on ENC (Enable Conversion) bit at the end of the configuration
    ADC12CTL0 |= ADC12ENC;

    return;
}

void whichADC(int touch){
    // Turn off ENC (Enable Conversion) bit while modifying the configuration
    ADC12CTL0 &= ~ADC12ENC;
    if(touch){
        // Undo detector
        ADC12MCTL2 &= ~ADC12VRSEL_1;
        ADC12MCTL2 &= ~(ADC12INCH1|ADC12EOS);                    // Lasor sensor

        ADC12MCTL0 &= ~(ADC12VRSEL3|ADC12VRSEL2|ADC12VRSEL1|ADC12VRSEL0);
        ADC12MCTL1 |= (ADC12INCH3|ADC12INCH2|ADC12INCH1|ADC12INCH0);//(ADC12INCH2|ADC12INCH1|ADC12EOS);                     // X
        ADC12MCTL0 |= (ADC12INCH3|ADC12INCH2|ADC12INCH1);//(ADC12INCH2|ADC12INCH1|ADC12INCH0);                   // Y


    }
    else{
        // Undo touch ADC
        ADC12MCTL1 &= ~(ADC12INCH3|ADC12INCH2|ADC12INCH1|ADC12INCH0|ADC12EOS);//(ADC12INCH2|ADC12INCH1|ADC12EOS);                     // X
        ADC12MCTL0 &= ~(ADC12INCH3|ADC12INCH2|ADC12INCH1);//(ADC12INCH2|ADC12INCH1|ADC12INCH0);                   // Y

        ADC12MCTL2 |= ADC12VRSEL_1;
        ADC12MCTL2 |= (ADC12INCH1|ADC12EOS);                    // Lasor sensor
    }
    // Turn on ENC (Enable Conversion) bit at the end of the configuration
    ADC12CTL0 |= ADC12ENC;
}

void setTouchADC(){
    whichADC(1);
}

void setDetectorADC(){
    whichADC(0);
}

void touch_initInterface(void)
{
    /* Initialize analog interface. */
    /* Initialize the ADC12_B Module. */

/*
    ADC12_B_init(ADC12_B_BASE,
                 ADC12_B_SAMPLEHOLDSOURCE_SC,
                 ADC12_B_CLOCKSOURCE_ADC12OSC,
                 ADC12_B_CLOCKDIVIDER_1);

    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_64_CYCLES,
                               ADC12_B_CYCLEHOLD_64_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);
    ADC12_B_enable(ADC12_B_BASE);

    ADC12_B_configureMemoryParam adc12MemConfigParamsY =
    {
        TOUCH_Y_PLUS_MEMORY,
        TOUCH_Y_PLUS_INPUT,
        ADC12_B_VREFPOS_AVCC,
        ADC12_B_VREFNEG_AVSS,
        ADC12_B_NOTENDOFSEQUENCE
    };

    ADC12_B_configureMemoryParam adc12MemConfigParamsX =
    {
        TOUCH_X_PLUS_MEMORY,
        TOUCH_X_PLUS_INPUT,
        ADC12_B_VREFPOS_AVCC,
        ADC12_B_VREFNEG_AVSS,
        ADC12_B_NOTENDOFSEQUENCE
    };

    /* Configure Y+ input to memory buffer 0. /
    ADC12_B_configureMemory(ADC12_B_BASE,
                            &adc12MemConfigParamsY);

    /* Configure X+ input to memory buffer 1. /
    ADC12_B_configureMemory(ADC12_B_BASE,
                            &adc12MemConfigParamsX);
*/

    /*ADC12_B_initParam initParam = {0};
        initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
        initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
        initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
        initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;

    ADC12_B_enable(ADC12_B_BASE);
    ADC12_B_init(ADC12_B_BASE, &initParam);


    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_64_CYCLES,
                               ADC12_B_CYCLEHOLD_64_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);

    __enable_interrupt();
    */
/*
    ADC12_B_configureMemoryParam adc12MemConfigParamsY =
    {
        TOUCH_Y_PLUS_MEMORY,
        TOUCH_Y_PLUS_INPUT,
        ADC12_B_VREFPOS_AVCC,
        ADC12_B_VREFNEG_AVSS,
        ADC12_B_NOTENDOFSEQUENCE
    };

    ADC12_B_configureMemoryParam adc12MemConfigParamsX =
    {
        TOUCH_X_PLUS_MEMORY,
        TOUCH_X_PLUS_INPUT,
        ADC12_B_VREFPOS_AVCC,
        ADC12_B_VREFNEG_AVSS,
        ADC12_B_NOTENDOFSEQUENCE
    };
*/
    // Parameters intended to be set to default
        //refVoltageSourceSelect
        //endOfSequence
        //windowComparatorSelect
        //differentialModeSelect

    ADC12_B_configureMemoryParam adc12MemConfigParamsY;
        //adc12MemConfigParamsY.memoryBufferControlIndex = TOUCH_Y_PLUS_MEMORY;
        //adc12MemConfigParamsY.inputSourceSelect = TOUCH_Y_PLUS_INPUT;

    ADC12_B_configureMemoryParam adc12MemConfigParamsX;
        //adc12MemConfigParamsX.memoryBufferControlIndex = TOUCH_X_PLUS_MEMORY;
        //adc12MemConfigParamsX.inputSourceSelect = TOUCH_X_PLUS_INPUT;

    /* Configure Y+ input to memory buffer 0. */
    ADC12_B_configureMemory(ADC12_B_BASE,
                            &adc12MemConfigParamsY);

    /* Configure X+ input to memory buffer 1. */
    ADC12_B_configureMemory(ADC12_B_BASE,
                            &adc12MemConfigParamsX);

    /* Configure SW1 for input. */
    /*GPIO_setAsInputPinWithPullUpResistor(TOUCH_SW1_PORT, TOUCH_SW1_PIN);

    /* Check if the screen has already been calibrated. */
    /*if(touch_calibrationData.key == TOUCH_CALIBRATION_KEY)
    {
        /* Return if the user is not manually requesting calibration. */
        /*if(GPIO_getInputPinValue(TOUCH_SW1_PORT, TOUCH_SW1_PIN))
        {
            return;
        }
    }*/

    /* Wait for SW1 to be released. */
    /*while(!GPIO_getInputPinValue(TOUCH_SW1_PORT, TOUCH_SW1_PIN))
    {
        ;
    }

    /* Calibrate the touch screen. */
    //touch_calibrate();
}

void calibration(touch_calibration* temp){
    temp->xMax = touch_calibrationData.xMax;
    temp->xMin= touch_calibrationData.xMin;
    temp->yMax = touch_calibrationData.yMax;
    temp->yMin = touch_calibrationData.yMin;
    return;
}

uint16_t scaleX(uint16_t adcResult){
    float q12Ratio;

    /* Map the analog reading to the display coordinates. */
    /* Map the ADC reading to the display coordinates. */
    q12Ratio =
        (float)(((float)(adcResult -
                         touch_calibrationData.xMin)) /
                (float)(touch_calibrationData.xMax -
                        touch_calibrationData.xMin));
    if(q12Ratio >= 1)
    {
        q12Ratio = 1;
    }
    if(q12Ratio < 0)
    {
        q12Ratio = 0;
    }
    return (uint16_t)(q12Ratio * LCD_HORIZONTAL_MAX);
}



uint16_t scaleY(uint16_t adcResult){
    float q12Ratio;

    /* Map the analog reading to the display coordinates. */
    /* Map the ADC reading to the display coordinates. */
    q12Ratio =
        (float)(((float)(adcResult -
                         touch_calibrationData.yMin)) /
                ((float)(touch_calibrationData.yMax -
                         touch_calibrationData.yMin)));
    if(q12Ratio >= 1)
    {
        q12Ratio = 1;
    }
    if(q12Ratio < 0)
    {
        q12Ratio = 0;
    }

    return (uint16_t)(q12Ratio * LCD_VERTICAL_MAX);
}

static void checkTemp(int temp){
    printf(temp);

    int temp2 = temp;
}

/*
 * Returns true when a touch is detected.
 */
void touch_detectedTouch(void){
    uint16_t ui16ADCtemp;

    // Ground Y-
    // ** DO THIS FIRST **
    // Doing it first ensures no toggling while configuring pins
    P4DIR |= TOUCH_Y_MINUS_PIN;
    P4OUT &= ~TOUCH_Y_MINUS_PIN;

    // Pull X- to Vcc
    P2DIR &= ~TOUCH_X_MINUS_PIN;
    P2REN |= TOUCH_X_MINUS_PIN;
    P2OUT |= TOUCH_X_MINUS_PIN;

    /* Set X- and Y- as output and Y+ as input (floating). */
    //GPIO_setAsOutputPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);
    //GPIO_setAsOutputPin(TOUCH_Y_MINUS_PORT, TOUCH_Y_MINUS_PIN);
    //GPIO_setAsInputPin(TOUCH_X_PLUS_PORT, TOUCH_X_PLUS_PIN);
    GPIO_setAsInputPin(TOUCH_Y_PLUS_PORT, TOUCH_Y_PLUS_PIN);
    /* Sample the X+ ADC channel to check if there is currently a touch. */
    GPIO_setAsPeripheralModuleFunctionInputPin(TOUCH_X_PLUS_PORT,
                                                TOUCH_X_PLUS_PIN,
                                                GPIO_TERNARY_MODULE_FUNCTION);

    /* Set X- high and Y- low to detect touch. */
    /*GPIO_setOutputHighOnPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);
    GPIO_setOutputLowOnPin(TOUCH_Y_MINUS_PORT, TOUCH_Y_MINUS_PIN);*/
    //GPIO_setAsInputPinWithPullUpResistor(TOUCH_X_PLUS_PORT, TOUCH_X_PLUS_PIN);  // Pullup Y+
    //P8REN |= TOUCH_X_PLUS_PIN;
    //P8OUT |= TOUCH_X_PLUS_PIN;
    // GPIO_setOutputLowOnPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);              // Ground X-

    /*ADC12_B_clearInterrupt(ADC12_B_BASE, 0, TOUCH_X_PLUS_IFG);
    ADC12_B_startConversion(ADC12_B_BASE, TOUCH_X_PLUS_MEMORY,
                            ADC12_B_SINGLECHANNEL);*/
    /* Wait for SW1 to be pressed. */
    /*
    while(GPIO_getInputPinValue(TOUCH_SW1_PORT, TOUCH_SW1_PIN))
    {
        volatile unsigned int temp = ADC12_B_getResults(ADC12_B_BASE, TOUCH_X_PLUS_MEMORY), temp2;
        ADC12MEM1 &= 0;
    }
    */

    //while(GPIO_getInputPinValue(TOUCH_Y_PLUS_PORT, TOUCH_Y_PLUS_PIN) != 0);
}

bool cal_touch_detectedTouch(void){
    uint16_t ui16ADCtemp;
    // Set up pins and flags
    touch_detectedTouch();
    _low_power_mode_3();

    //P2IFG &= ~TOUCH_X_MINUS_PIN;    // Clear flag
    return ((P2IN & TOUCH_X_MINUS_PIN) == 0);
    ADC12MEM1 =0;
    ADC12CTL0 |= ADC12SC;

    // Wait for conversion to finish
    while(!ADC12_B_getInterruptStatus(ADC12_B_BASE, 0,TOUCH_X_PLUS_IFG));

    // Pull reading
    ui16ADCtemp = ADC12MEM1;

    // Clear memory flag
    ADC12IFGR0 &= ~(TOUCH_X_PLUS_IFG);
    volatile unsigned int temp2 = ui16ADCtemp;
/*
    if(4095 < 350)
    {
        temp = 1;
    }
    else
    {
        temp = 0;
    }
*/
    /* Check if the detected touch is below the threshold. */
    temp2 = (temp2 < TOUCH_THRESHOLD);
    return (temp2);
}

uint16_t touch_sampleX(void)
{
    uint32_t average = 0;
    uint8_t i;
    /* Set X+ and X- as output and Y- as input (floating). */
    GPIO_setAsOutputPin(TOUCH_X_PLUS_PORT, TOUCH_X_PLUS_PIN);
    GPIO_setAsOutputPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);
    GPIO_setAsInputPin(TOUCH_Y_MINUS_PORT, TOUCH_Y_MINUS_PIN);

    /* Set X+ high and X- low. */
    GPIO_setOutputHighOnPin(TOUCH_X_PLUS_PORT, TOUCH_X_PLUS_PIN);
    GPIO_setOutputLowOnPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);

    /* Sample the Y+ ADC channel. */
    GPIO_setAsPeripheralModuleFunctionOutputPin(TOUCH_Y_PLUS_PORT,
                                                TOUCH_Y_PLUS_PIN,
                                                GPIO_TERNARY_MODULE_FUNCTION);


    // Clear memory flag
    ADC12IFGR0 &= ~(TOUCH_Y_PLUS_IFG);
    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, TOUCH_X_PLUS_IFG | TOUCH_Y_PLUS_IFG);

    char send[1];
    for(i = 0; i < TOUCH_OVERSAMPLE; i++)
    {
        // Start conversions
        ADC12CTL0 |= ADC12SC;
        //ADC12_B_startConversion(ADC12_B_BASE, TOUCH_Y_PLUS_MEMORY,
          //                      ADC12_B_SINGLECHANNEL);
//        ADC12IFGR0 &= ~(TOUCH_Y_PLUS_IFG);
        while((ADC12IFGR0 & TOUCH_Y_PLUS_IFG) == 0){
        }
        send[0] = ('1' + i);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);

        average += ADC12MEM0;
        ADC12MEM0 = 0;
        ADC12IFGR0 &= ~(TOUCH_Y_PLUS_IFG);
    }

    /* Return the analog result. */
    return(average >> TOUCH_AVERAGE_DIVISOR);
}

/*
 * Sample the Y analog axis.
 */
uint16_t touch_sampleY(void)
{
    uint32_t average = 0;
    uint8_t i;
    /* Set Y+ and Y- as output and X- as input (floating). */
    GPIO_setAsOutputPin(TOUCH_Y_PLUS_PORT, TOUCH_Y_PLUS_PIN);
    GPIO_setAsOutputPin(TOUCH_Y_MINUS_PORT, TOUCH_Y_MINUS_PIN);
    GPIO_setAsInputPin(TOUCH_X_MINUS_PORT, TOUCH_X_MINUS_PIN);

    /* Set Y+ low and Y- high. */
    GPIO_setOutputLowOnPin(TOUCH_Y_PLUS_PORT, TOUCH_Y_PLUS_PIN);
    GPIO_setOutputHighOnPin(TOUCH_Y_MINUS_PORT, TOUCH_Y_MINUS_PIN);

    /* Sample the X+ ADC channel. */
    GPIO_setAsPeripheralModuleFunctionOutputPin(TOUCH_X_PLUS_PORT,
                                                TOUCH_X_PLUS_PIN,
                                                GPIO_TERNARY_MODULE_FUNCTION);

    // Clear memory flag
    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, TOUCH_X_PLUS_IFG | TOUCH_Y_PLUS_IFG);
    ADC12IFGR1 &= ~(TOUCH_X_PLUS_IFG);

    for(i = 0; i < TOUCH_OVERSAMPLE; i++)
    {
        // Start conversions
        ADC12CTL0 |= ADC12SC;
        /*ADC12_B_startConversion(ADC12_B_BASE, TOUCH_X_PLUS_MEMORY,
                                ADC12_B_SINGLECHANNEL);*/
        while((ADC12IFGR0 & TOUCH_X_PLUS_IFG) == 0){

        }

        /*while(!ADC12_B_getInterruptStatus(ADC12_B_BASE, 0, TOUCH_X_PLUS_IFG))
        {
            ;
        }*/
        average += ADC12MEM1;
    }

    /* Return the anlog result. */
    return(average >> TOUCH_AVERAGE_DIVISOR);
}

static void touch_calibrateCircle(uint16_t *xSum,
                                  uint16_t *ySum,
                                  uint16_t xPos,
                                  uint16_t yPos)
{
    /* Draw a red circle for the user to touch. */
    Graphics_setForegroundColor(&g_sContext, ClrRed);
    Graphics_fillCircle(&g_sContext, xPos, yPos, TOUCH_CALIBRATION_RADIUS);

    /* Wait for a tocuh to be detected and wait ~4ms. */
    cal_touch_detectedTouch();

    // Toggle edge-signal (Assumption: all touches are valid)
    P2IES ^= TOUCH_X_MINUS_PIN;

    __delay_cycles(100000);

    /* Sample the X and Y measurements of the touch screen. */
    *xSum += touch_sampleX();
    *ySum += touch_sampleY();


    // Pre-caution: clear flag
    P2IFG &= ~TOUCH_X_MINUS_PIN;

    /* Wait for the touch to stop and wait ~4ms. */
    while(cal_touch_detectedTouch())
    {
        ;
    }

    // Control: set X- to high
    P2OUT |= TOUCH_X_MINUS_PIN;
    P2DIR |= TOUCH_X_MINUS_PIN;
    // Toggle edge-signal (Assumption: all touches are valid)
    P2IES ^= TOUCH_X_MINUS_PIN;
    // Pre-caution: clear flag
    P2IFG &= ~TOUCH_X_MINUS_PIN;


    __delay_cycles(100000);

    /* Clear the drawn circle. */
    Graphics_setForegroundColor(&g_sContext, ClrBlack);
    Graphics_fillCircle(&g_sContext, xPos, yPos, TOUCH_CALIBRATION_RADIUS);

    return;
}

void touch_calibrate(void)
{
    touch_calibration calData;

    //FRAMCtl_unlockInfoA();
    //FRAMCtl_eraseSegment((uint8_t *)&touch_calibrationData);

    /* Zero out local copies of calibration data. */
    calData.xMin = 0;
    calData.xMax = 0;
    calData.yMin = 0;
    calData.yMax = 0;

    /* Display Instructions to the screen. */
    Graphics_setForegroundColor(&g_sContext, ClrWhite);
    Graphics_drawStringCentered(&g_sContext,
                                "TOUCH DOT TO CALIBRATE",
                                AUTO_STRING_LENGTH,
                                LCD_HORIZONTAL_MAX / 2,
                                LCD_VERTICAL_MAX / 2,
                                TRANSPARENT_TEXT);

    /* Top left corner. */
    touch_calibrateCircle(
        &calData.xMin,
        &calData.yMin,
        TOUCH_CALIBRATION_RADIUS,
        TOUCH_CALIBRATION_RADIUS);

    /* Top right corner. */
    touch_calibrateCircle(
        &calData.xMax,
        &calData.yMin,
        (LCD_HORIZONTAL_MAX - 1) - TOUCH_CALIBRATION_RADIUS,
        TOUCH_CALIBRATION_RADIUS);

    /* Bottom left corner. */
    touch_calibrateCircle(
        &calData.xMin,
        &calData.yMax,
        TOUCH_CALIBRATION_RADIUS,
        (LCD_VERTICAL_MAX - 1) - TOUCH_CALIBRATION_RADIUS);

    /* Bottom right corner. */
    touch_calibrateCircle(
        &calData.xMax,
        &calData.yMax,
        (LCD_HORIZONTAL_MAX - 1) - TOUCH_CALIBRATION_RADIUS,
        (LCD_VERTICAL_MAX - 1) - TOUCH_CALIBRATION_RADIUS);

    /* Compensate for the radius offset to caluculate final X calibration values. */
    calData.xMin = (calData.xMin >> 1) - TOUCH_CALIBRATION_RADIUS;
    calData.xMax = (calData.xMax >> 1) + TOUCH_CALIBRATION_RADIUS;

    /* Compensate for the radius offset to caluculate final Y calibration values. */
    calData.yMin = (calData.yMin >> 1) - TOUCH_CALIBRATION_RADIUS;
    calData.yMax = (calData.yMax >> 1) + TOUCH_CALIBRATION_RADIUS;

    /* Write the calibration key to signal the values have been saved. */
    calData.key = TOUCH_CALIBRATION_KEY;

    /* Save the calibration data. */
    touch_calibrationData.xMax = calData.xMax;
    touch_calibrationData.xMin = calData.xMin;
    touch_calibrationData.yMax = calData.yMax;
    touch_calibrationData.yMin = calData.yMin;
    volatile touch_calibration tempC;

    // Disable MPU to allow writing to non-volitile memory
    MPUCTL0 = MPUPW;
    MPUSAM = (MPUSEG1WE | MPUSEG2WE | MPUSEG3WE | MPUSEG1RE | MPUSEG2RE | MPUSEG3RE | MPUSEG1XE | MPUSEG2XE | MPUSEG3XE);

    FRAMCtl_write32((uint32_t *)&calData, (uint32_t *)&touch_calibrationData,
                         sizeof(touch_calibrationData) / 4);
    // Re-enable MPU to protect data
    MPUCTL0 = MPUPW | MPUENA;

    //FRAMCtl_lockInfoA();
    tempC = touch_calibrationData;

    Graphics_clearDisplay(&g_sContext);
}
