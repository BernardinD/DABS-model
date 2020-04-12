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
#include <msp430fr6989.h>
#include <touch_FR6989.h>
#include "grlib.h"
#include "radioButton.h"
#include "checkbox.h"
#include "driverlib.h"
#include "kitronix320x240x16_ssd2119_spi.h"
#include "HAL_MSP_EXP432P401R_KITRONIX320X240_SSD2119_SPI.h"
#include "Images/images.h"
#include "menus.h"


// Graphic library context
Graphics_Context g_sContext;

//Flag to know if a demo was run
bool g_ranDemo = false;

// LEDs
#define redLED BIT0 // Red at P1.0
#define greenLED BIT7 // Green at P9.7

// Buttons
#define BUT1 BIT1 // Button S1 at Port 1.1
#define BUT2 BIT2 // Button S2 at Port 1.2

void Delay();
void boardInit(void);
//void clockInit(void);
void timerInit(void);
void initializeButtons(void);
void drawMainMenu(void);
void Initialize_modules_pins();
uint16_t diff(uint16_t, uint16_t);
volatile uint8_t laser1 = 0;
extern int textHeight = 50;
double final_result;
int sleepCount = 0, touch_cal_count = 0;
extern void eraseButton(Graphics_Button* button);

void config_ACLK_to_32KHz_crystal();

#if defined(__IAR_SYSTEMS_ICC__)
int16_t __low_level_init(void) {
    // Stop WDT (Watch Dog Timer)
    WDTCTL = WDTPW + WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;         // Disable GPIO power-on default high-impedance mode

    // Configure SMCLK to 8 MHz (used as SPI clock)
        CSCTL0 = CSKEY;                 // Unlock CS registers
        CSCTL3 &= ~(BIT4|BIT5|BIT6);    // DIVS=0
        CSCTL0_H = 0;                   // Relock the CS registers
    return(1);
}

#endif

volatile char mystring[30];
volatile int8_t cal = 1;
void setCal(uint8_t new){
    cal = new;
}

// To use for dropping redundant points
volatile int16_t prevX = 10000;
volatile int16_t prevY = 10000;

// Beer values
double good = 0.1, bad = 0.02;
// 0.5, 1.4

// All changes to list should reflect in function 'finalize()'
int8_t* our_recommendations[] = {
                                 "*** RECOMMENDATION ERROR. THEY HAVE NOT BEEN SET ***",
                                 "Your results have come back as good.,"
                                 "No recommendations.",
                                 "Your results have come back as ok.",
                                 "Your results have come back as bad.",
                                 "Seek immediate medical attention."
};
int recomm_num= 0;
//extern recomm_num;

void main(void)
{


    // Stop WDT (Watch Dog Timer)
    WDTCTL = WDTPW + WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;         // Disable GPIO power-on default high-impedance mode

    //SET REFERENCE
    //while(REFCTL0 & REFGENBUSY);                            // If ref generator busy, WAIT
    //REFCTL0 |= REFVSEL_1 | REFON;                           // Select internal ref = 2.5V (For the ADC) and Internal Reference ON


    // Configure SMCLK to 8 MHz (used as SPI clock)
    CSCTL0 = CSKEY;                 // Unlock CS registers
    CSCTL3 &= ~(BIT4|BIT5|BIT6);    // DIVS=0
    CSCTL0_H = 0;                   // Relock the CS registers

    /* Set up screen low-power mode test (using button interrupt)*/
    P1DIR |= redLED; // Pins as output
    P9DIR |= greenLED;
    P1OUT &= ~redLED; // Red off
    P9OUT &= ~greenLED; // Green o

    // Configuring buttons with interrupt
       /* P1DIR &= ~(BUT1|BUT2);      // 0: input
        P1REN |= (BUT1|BUT2);       // 1: enable built-in resistors
        P1OUT |= (BUT1|BUT2);       // 1: built-in resistor is pulled up to Vcc
        P1IE  |= (BUT1|BUT2);       // 1: enable interrupts
        P1IES |= (BUT1|BUT2);       // 1: interrupt on falling edge
        P1IFG &= ~(BUT1|BUT2);      // 0: clear the interrupt flags*/

    // Enable the global interrupt bit (call an intrinsic function)
        _enable_interrupts();

    // Initialization routines
    boardInit();
    //clockInit();
    //config_ACLK_to_32KHz_crystal();
    timerInit();
    initializeDemoButtons();
    initializeButtons();

    // LCD setup using Graphics Library API calls
    Kitronix320x240x16_SSD2119Init();
    Graphics_initContext(&g_sContext, &g_sKitronix320x240x16_SSD2119);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setFont(&g_sContext, &g_sFontCmss20b);
    //Graphics_clearDisplay(&g_sContext);

    // Diagnostics and loading screen
    run_startup();


    // Turn off calibration screen
    setCal(0);


    updateMenuScreen(0);           // Goes to main menu

    // Set up pins and flags
    P2IES |= TOUCH_X_MINUS_PIN;     // Set to down-edge signal
    touch_detectedTouch();
    /*int *foo1, *foo2;
    while(1)
        run(foo1,foo2);*/
    while(1){
        _low_power_mode_3();

        if(cal){
            touch_calibrate();
        }
    }
}

void run_startup(){

    Initialize_ADC();
    // Increase loading bar
    //rect.xMax += (int)(barWidth*0.20);
    //Graphics_fillRectangle(&g_sContext, &rect);

    Initialize_touch();
    // Increase loading bar
    //rect.xMax += (int)(barWidth*0.20);
    //Graphics_fillRectangle(&g_sContext, &rect);

    // Draw loading screen
    /* Display Text to the screen. */
    Graphics_setForegroundColor(&g_sContext, ClrWhite);
    Graphics_drawStringCentered(&g_sContext,
                                "System starting up",
                                AUTO_STRING_LENGTH,
                                LCD_HORIZONTAL_MAX / 2,
                                LCD_VERTICAL_MAX / 2,
                                TRANSPARENT_TEXT);

    /* Draw loading bar */
    volatile int barHeight = (int)(LCD_HORIZONTAL_MAX*0.10),
            barWidth = (int)(LCD_HORIZONTAL_MAX * 0.40),
            xMin = (LCD_HORIZONTAL_MAX / 2) - (barWidth/2),
            yMin = ((LCD_VERTICAL_MAX / 2) + (int)(LCD_VERTICAL_MAX * 0.20)) - (barHeight/2),
            xMax = (LCD_HORIZONTAL_MAX / 2) + (barWidth/2),
            yMax = ((LCD_VERTICAL_MAX / 2) + (int)(LCD_VERTICAL_MAX * 0.20)) + (barHeight/2);
    // Rect = xMin, yMin, xMax, yMax
    Graphics_Rectangle rect = {xMin, yMin, xMax, yMax};
    Graphics_drawRectangle(&g_sContext, &rect);

    rect.xMax = rect.xMin;


    // Initialize remaining components
    Initialize_modules_pins();

    // Test Fans
    // ...
    // Increase loading bar
    rect.xMax += (int)(barWidth*0.20);
    Graphics_fillRectangle(&g_sContext, &rect);

    // Test laser
    // ...
    // Increase loading bar
    rect.xMax += (int)(barWidth*0.20);
    Graphics_fillRectangle(&g_sContext, &rect);

    /* Fill up rest of bar */
    // Increase loading bar
    rect.xMax += (int)(barWidth*0.20);
    Graphics_fillRectangle(&g_sContext, &rect);


}

void Initialize_modules_pins(){
    // Fans 1&2 - Output
    P2DIR |= (BIT1 | BIT2);
    P2OUT &= ~(BIT1 | BIT2); // Fans off

    // Laser signals - Output
    P6DIR |= (BIT0 | BIT1);
    P6OUT &= ~(BIT0 | BIT1); // Lasers off

    // Sensor signal - Output
    P3DIR |= BIT7;
    P3OUT &= ~BIT7;         // Temp start signal off

    // Sensor ADC reading, -- ALL OTHER ADC SETTING SHOULD BE TAKEN CARE OF BY TOUCHSCREEN --
    /*P1SEL1 |= BIT2;
    P1SEL0 |= BIT2;
    // Turn off ENC (Enable Conversion) bit while modifying the configuration
    ADC12CTL0 &= ~ADC12ENC;
    ADC12MCTL3 |= ADC12VRSEL_1;
    ADC12MCTL3 |= (ADC12INCH1|ADC12EOS);                     // Lasor sensor
    // Turn on ENC (Enable Conversion) bit at the end of the configuration
    ADC12CTL0 |= ADC12ENC;*/

}
uint16_t diff(uint16_t a, uint16_t b){
    int16_t d = a - b;
    return (d>0)?d:(-1)*d;
}

// Only the interrupt for the touchscreen pin should be enabled
#pragma vector = PORT1_VECTOR
__interrupt void mine() {

    if((P1IFG & BUT1) != 0)
    {    // Toggle the red LED
        if ((P9OUT & greenLED)==0)  // If green light off
            GPIO_setOutputHighOnPin(LCD_PWM_PORT,
                                        LCD_PWM_PIN);
        if ((P9OUT & greenLED)!=0)  // If green light on
            GPIO_setOutputLowOnPin(LCD_PWM_PORT,
                                        LCD_PWM_PIN);
        P9OUT ^= greenLED;
        P1OUT ^= redLED;
        P1IFG &= ~BUT1;
    }
    if((P1IFG & BUT2) != 0){
        P1IFG &= ~BUT2;
    }
}

// Used to deactivate low-power mode when signaling touch response
#pragma vector = PORT2_VECTOR
__interrupt void mine2() {
    if(cal){
        __low_power_mode_off_on_exit();
    }
    else{
        // If touch
        //setSleepTimer();
            // Exit sleep mode
        if ((P2OUT & LCD_PWM_PIN) == 0){
            GPIO_setOutputHighOnPin(LCD_PWM_PORT,
                                        LCD_PWM_PIN);
        }
            // Start timer to retrieve touch coordinates, \
                -Flip signal edge
        else{
            // If touch-down
            if((P2IN & TOUCH_X_MINUS_PIN)==0){
                /*Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_drawStringCentered(&g_sContext, "Touched down ",
                                                        AUTO_STRING_LENGTH,
                                                        (LCD_HORIZONTAL_MAX/2),
                                                        (LCD_VERTICAL_MAX/2),
                                                        TRANSPARENT_TEXT);*/
                // Toggle edge-signal
                P2IES &= ~TOUCH_X_MINUS_PIN;

                TA1CCTL1 |= CCIFG;                // Set timer flag, so to immediately take points
                TA1CTL |= (TACLR);              // Enable interrupts and clear timer
                TA1CCTL1 |= CCIE;
            }
            // If release,
                // -Flip signal edge \
                    -Enable pins for touch detection \
                    -Disable timer interrupt \
                    -Turn off low-power mode
            else{
                // If touch-up
                // Toggle edge-signal
                P2IES |= TOUCH_X_MINUS_PIN;

                // Control: set X- to high
                P2OUT |= TOUCH_X_MINUS_PIN;
                P2DIR |= TOUCH_X_MINUS_PIN;

                touch_detectedTouch();

                TA1CCTL1 &= ~CCIE;
                __low_power_mode_off_on_exit();
            }
        }
    }
    // Clear flag
    P2IFG &= ~TOUCH_X_MINUS_PIN;
}

double beer(double sample, double test){
    return 1e7;
}

// ISR of Timer (A0 vector)
// Channel 0
#pragma vector = TIMER1_A0_VECTOR
__interrupt void T1A0_ISR() {

    // Using touch pin as flag since it should be on during most operations
    // and off during the laser operation
    /*if((P2IE & TOUCH_X_MINUS_PIN) != 0){
        // But device to sleep
        // Check if count == limit, if so sleep, else increase counter
        if (sleepCount == 8){
            putToSleep();
            TA1CCTL0 &= ~CCIE;
        }
        else
            sleepCount += 1;

    }*/
    //else{
        // Using for delay in laser on/off
        __low_power_mode_off_on_exit();
//    }
}

// ISR of Timer (A1 vector)
// Rollback and Ch. 1&2
#pragma vector = TIMER1_A1_VECTOR
__interrupt void T1A_ISR() {

    /*Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_drawStringCentered(&g_sContext, "Touched down ",
                                            AUTO_STRING_LENGTH,
                                            (LCD_HORIZONTAL_MAX/2),
                                            (LCD_VERTICAL_MAX/2),
                                            TRANSPARENT_TEXT);*/

    // If finger is currently down \
        (just in case timer interrupt has higher priority \
        than the GPIO interrupt)
    //if ((P2IN & TOUCH_X_MINUS_PIN) == 0){
    // Take points
    volatile uint16_t x = touch_sampleX();
    volatile uint16_t y = touch_sampleY();


    /*Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    Graphics_drawStringCentered(&g_sContext, "Pulled points ",
                                            AUTO_STRING_LENGTH,
                                            (LCD_HORIZONTAL_MAX/2),
                                            (LCD_VERTICAL_MAX/2),
                                            TRANSPARENT_TEXT);*/
    x = scaleX(x);
    y = scaleY(y);

    // If x || y have value at the extreme more that 4 times, exit low power mode to
    // recalibrate
    /*if(x >= LCD_HORIZONTAL_MAX || y >= LCD_VERTICAL_MAX){
        if(touch_cal_count > 4){
            // Draw warning for recalibration
            Graphics_Rectangle clear = {0,textHeight+15,LCD_HORIZONTAL_MAX -10,LCD_VERTICAL_MAX -10};
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_fillRectangle(&g_sContext, &clear);

            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
            Graphics_drawStringCentered(&g_sContext, "Calibration error detected",
                                        AUTO_STRING_LENGTH,
                                        159,
                                        textHeight,
                                        TRANSPARENT_TEXT);
            Graphics_drawStringCentered(&g_sContext, "Tap again to confirm.",
                                        AUTO_STRING_LENGTH,
                                        159,
                                        textHeight + 25,
                                        TRANSPARENT_TEXT);

            // Check one more time
            setCal(1);        // For function's LPM call
            /* Wait for a tocuh to be detected and wait ~4ms. */
     /*       cal_touch_detectedTouch();

            x = touch_sampleX();
            y = touch_sampleY();
            x = scaleX(x);
            y = scaleY(y);

            /* Wait for the touch to stop and wait ~4ms. */
      /*      while(cal_touch_detectedTouch())
                {
                    ;
                }

            // Clear text
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawStringCentered(&g_sContext, "Calibration error detected",
                                        AUTO_STRING_LENGTH,
                                        159,
                                        textHeight,
                                        TRANSPARENT_TEXT);
            Graphics_drawStringCentered(&g_sContext, "Tap again to confirm.",
                                        AUTO_STRING_LENGTH,
                                        159,
                                        textHeight + 25,
                                        TRANSPARENT_TEXT);


            // If fail, return and recalibrate
            if(x >= LCD_HORIZONTAL_MAX || y >= LCD_VERTICAL_MAX){


                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                Graphics_drawStringCentered(&g_sContext, "Initiating recalibration.",
                                            AUTO_STRING_LENGTH,
                                            159,
                                            textHeight,
                                            TRANSPARENT_TEXT);

                // Return signal to recalibrate
                return 0;

                // Turn off touch screen
                // ...
            }
            // Else, return to home and restart
            else{
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                Graphics_drawStringCentered(&g_sContext, "Everything is fine,",
                                            AUTO_STRING_LENGTH,
                                            159,
                                            textHeight,
                                            TRANSPARENT_TEXT);
                Graphics_drawStringCentered(&g_sContext, "returning to home.",
                                            AUTO_STRING_LENGTH,
                                            159,
                                            textHeight + 25,
                                            TRANSPARENT_TEXT);
                updateMenuScreen(getCurrMenuScreen());
                return 1;
            }
        }
    }
    else{
        touch_cal_count = 0;
    }*/

//    volatile uint16_t x = scaleX(touch_sampleX());
//    volatile uint16_t y = scaleY(touch_sampleY());

    volatile uint16_t diffX = diff(prevX, x);
    volatile uint16_t diffY = diff(prevY, y);
    /*if((diffX < (LCD_HORIZONTAL_MAX*0.05)) && (diffY < (LCD_VERTICAL_MAX*0.05)))
        TA1CTL &= ~TAIFG;
        return;*/

    prevX = x;
    prevY = y;

    // Configure pins to detect release
    touch_detectedTouch();

    switch(getCurrMenuScreen()/10){
        // Main menu
        case (0):
            mainMenu(x, y);
            break;
        // Settings
        case (1):
            settingsMenu(x, y);
            break;
        // Between page
        case (2):
            continuingMenu(x, y);
            break;
        // Results page
        case (3):
            resultsMenu(x, y);
            break;
        // Default demo
        case (99):
            runRestarDemo(x,y);
            break;
        default:
            break;
    }
    //}

    // Clear flag
    TA1CCTL1 &= ~CCIFG;

}


void config_ACLK_to_32KHz_crystal() {
    // By default, ACLK runs on LFMODCLK at 5MHz/128 = 39 KHz

    // Reroute pins to LFXIN/LFXOUT functionality
    PJSEL1 &= ~BIT4;
    PJSEL0 |= BIT4;

    // Wait until the oscillator fault flags remain cleared
    CSCTL0 = CSKEY;             // Unlock CS registers
    do {
    CSCTL5 &= ~LFXTOFFG; // Local fault flag
    SFRIFG1 &= ~OFIFG; // Global fault flag
    } while((CSCTL5 & LFXTOFFG) != 0);
    CSCTL0_H = 0; // Lock CS registers
    return;
}

// Function to switch between delay timers
void setWhichTimer(int laser){

    if (laser){
        TA1CCR0 = (32000)/8 * 4;      // Set interval to 4 sec
        // Reset flags for other timer (count)
    }
    else{
        // Set timer interval (8 sec)
        TA1CCR0 = 32000;

        // Turn on timer
        //--- Restart timer ---
        TA1CTL |= TACLR;
        //--- Clear flag ---
        TA1CCTL0 &= ~TAIFG;
        //--- Enable interrupt for channel 0---
        TA1CCTL0 |= CCIE;
    }
    sleepCount = 0;
}


void setLaserTimer(){
    setWhichTimer(1);
}

void setSleepTimer(){
    setWhichTimer(0);
}

void timerInit(){
    // Touch timer
    // Configure Channel 0 & 1 for up mode with interrupt
    // Timer for delays (defaults to sleep timer)
    //setSleepTimer();
    TA1CCR0 = (32000)/8 * 4;      // Set interval to 4 sec

    // Timer for touch intervals
    TA1CCR1 = (32000)/8 * 0.5;      // Set interval to 4 sec

    // Timer_A configuration (fill the line below)
    // Use ACLK, divide by 8, Up mode
    TA1CTL = TASSEL_1 | ID_3 | MC_1;
}


void drawMainMenu(void)
{
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext, "Welcome to DABS ^-^",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);

    // Draw sleep button
    Graphics_drawButton(&g_sContext, &sleepButton);

    // Draw TI banner at the bottom of screnn
    /*Graphics_drawImage(&g_sContext,
                       &TI_platform_bar_red4BPP_UNCOMP,
                       0,
                       Graphics_getDisplayHeight(
                           &g_sContext) - TI_platform_bar_red4BPP_UNCOMP.ySize);*/

    // Draw Primitives image button
    Graphics_drawButton(&g_sContext, &settingsButton);

    // Draw Images image button
    Graphics_drawButton(&g_sContext, &opButton);

}

void drawInBetween(void)
{
    // Get rid of text from last screen
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_drawStringCentered(&g_sContext, "Running sample. One moment...",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);


    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_drawStringCentered(&g_sContext, "Would you like to continue?",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);

    // Save a temporary homeButton in 'menu.c'
    saveHome();
    // Change the position of the home button for this page
    homeButton.text = "Restart";
    homeButton.xMin = 70;
    homeButton.xMax = 150;
    homeButton.yMin = 80;
    homeButton.yMax = 120;
    homeButton.textXPos = 80;
    homeButton.textYPos = 90;
    // Go back home for restarting
    Graphics_drawButton(&g_sContext, &homeButton);

    // Save a temporary opButton in 'menu.c'
    saveOp();
    opButton.text = "Continue";
    opButton.xMin = 180;
    opButton.xMax = 260;
    opButton.textXPos = 190;
    // Re-draw operation button
    Graphics_drawButton(&g_sContext, &opButton);

}

void drawSettings(void)
{
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext, "DABS Settings",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);

    // Draw Primitives image button
    Graphics_drawButton(&g_sContext, &calibrateButton);

    // Draw Images image button
    Graphics_drawButton(&g_sContext, &homeButton);
}

void drawResults(void){
    // Get rid of text from last screen
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_drawStringCentered(&g_sContext, "Running sample. One moment...",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);

    eraseButton(&sleepButton);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_drawStringCentered(&g_sContext, "Sampling Complete.",
                                AUTO_STRING_LENGTH,
                                159,
                                (LCD_VERTICAL_MAX / 2) - 25,
                                TRANSPARENT_TEXT);
    Graphics_drawStringCentered(&g_sContext, "Computing results...",
                                AUTO_STRING_LENGTH,
                                159,
                                (LCD_VERTICAL_MAX / 2),
                                TRANSPARENT_TEXT);

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_clearDisplay(&g_sContext);

    // Save a temporary homeButton in 'menu.c'
    saveHome();
    // Change the position of the home button for this page
    homeButton.text = "Home";
    homeButton.xMin = 0;
    homeButton.xMax = 60;
    homeButton.yMin = sleepButton.yMin;
    homeButton.yMax = sleepButton.yMax;
    homeButton.textXPos = 10;
    homeButton.textYPos = sleepButton.textYPos;
    // Go back home for restarting
    Graphics_drawButton(&g_sContext, &homeButton);


    Graphics_drawButton(&g_sContext, &sleepButton);


    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_drawStringCentered(&g_sContext, "Here are the results. What next?",
                                AUTO_STRING_LENGTH,
                                159,
                                textHeight,
                                TRANSPARENT_TEXT);

    // Clear banner at the bottom
    //Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_Rectangle clear = {0,200,LCD_HORIZONTAL_MAX,LCD_VERTICAL_MAX};
    //Graphics_drawRectangle(&g_sContext, &clear);

    // Draw results dialog box
    // Change font of context and save old
    Graphics_Font *temp_font = g_sContext.font;
    Graphics_setFont(&g_sContext, results.font);

    uint8_t *temp_text = results.text;
    results.text = "";
    Graphics_drawButton(&g_sContext, &results);
    results.text = temp_text;
    Graphics_setForegroundColor(&g_sContext, results.textColor);
    Graphics_drawStringCentered(&g_sContext, results.text,
                                AUTO_STRING_LENGTH,
                                results.textXPos,
                                results.textYPos,
                                TRANSPARENT_TEXT);


    // Draw recommendation(s) text box
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    clear.xMin=10;
    clear.yMin=150;
    clear.xMax=LCD_HORIZONTAL_MAX-10;
    clear.yMax=LCD_VERTICAL_MAX-10;
    Graphics_fillRectangle(&g_sContext, &clear);

    // Write recommendations
    int i;

    Graphics_setFont(&g_sContext, &g_sFontCm14);
        //volatile char c[] = our_recommendations[0];
        volatile char c2[] = "TEsting";
    switch(recomm_num/10){
        case(0):
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            Graphics_drawStringCentered(&g_sContext, our_recommendations[0],
                                            AUTO_STRING_LENGTH,
                                            (LCD_HORIZONTAL_MAX/2),
                                            150 + 10,
                                            TRANSPARENT_TEXT);
            break;
        default:
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            int start = recomm_num/10, length =((recomm_num/10) + (recomm_num%10) );
            for(i = start; i < length; i++)
                Graphics_drawStringCentered(&g_sContext, our_recommendations[i],
                                                AUTO_STRING_LENGTH,
                                                (LCD_HORIZONTAL_MAX/2),
                                                150 + i*10,
                                                TRANSPARENT_TEXT);
            break;
    }
    // Reset font of context
    Graphics_setFont(&g_sContext, temp_font);

    recomm_num = 0;
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    //write_recommendations();

}

void write_recommendations(void){

    int i;

    switch(recomm_num/10){
        /*case(1):
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            Graphics_drawStringCentered(&g_sContext, recommendations[0],
                                            AUTO_STRING_LENGTH,
                                            (LCD_HORIZONTAL_MAX/2),
                                            150 + 10,
                                            TRANSPARENT_TEXT);
            break;
        case(2):
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            int start = recomm_num%10, length =
            for(i = (recomm_num/10); i < ((recomm_num/10) + (recomm_num%10) ); i++)
                Graphics_drawStringCentered(&g_sContext, recommendations[],
                                                AUTO_STRING_LENGTH,
                                                (LCD_HORIZONTAL_MAX/2),
                                                150 + i*10,
                                                TRANSPARENT_TEXT);
            break;
        case(3):
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            for(i = (recomm_num/10); i < ((recomm_num/10) + (recomm_num%10) ); i++)
                Graphics_drawStringCentered(&g_sContext, recommendations[0],
                                                AUTO_STRING_LENGTH,
                                                (LCD_HORIZONTAL_MAX/2),
                                                150 + i*10,
                                                TRANSPARENT_TEXT);
            break;
        default:
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            for(i = 0; i < recomm_len; i++)
                Graphics_drawStringCentered(&g_sContext, recommendations[0],
                                                AUTO_STRING_LENGTH,
                                                (LCD_HORIZONTAL_MAX/2),
                                                150 + i*10,
                                                TRANSPARENT_TEXT);
            break;
        case(0):
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            Graphics_drawStringCentered(&g_sContext, our_recommendations[0],
                                            AUTO_STRING_LENGTH,
                                            (LCD_HORIZONTAL_MAX/2),
                                            150 + 10,
                                            TRANSPARENT_TEXT);
            break;
        default:
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            int start = recomm_num/10, length =((recomm_num/10) + (recomm_num%10) );
            for(i = start; i < length; i++)
                Graphics_drawStringCentered(&g_sContext, our_recommendations[i],
                                                AUTO_STRING_LENGTH,
                                                (LCD_HORIZONTAL_MAX/2),
                                                150 + i*10,
                                                TRANSPARENT_TEXT);
            break;

        recomm_num = 0;*/
    }
}

void boardInit(void)
{
    // Setup XT1 and XT2
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P5,
        GPIO_PIN2 + GPIO_PIN3 +
        GPIO_PIN4 + GPIO_PIN5,
        GPIO_PRIMARY_MODULE_FUNCTION
        );
}

void Delay(){
    __delay_cycles(SYSTEM_CLOCK_SPEED/10);
}
