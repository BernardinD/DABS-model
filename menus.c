#include "menus.h"
#include "touch_FR6989.h"
#include <math.h>

// ---- Constants ----
// LEDs
#define redLED BIT0 // Red at P1.0
#define greenLED BIT7 // Green at P9.7

// --- Variables ----
uint16_t menu = 1000;
Graphics_Button homeTemp;
Graphics_Button opTemp;
int midButtonYmax = 120, highButtonYmax= 35, buttonHeight = 40;
volatile unsigned int test_color1, test_color2, sample_color1, sample_color2;
int empty = 0;
//*(test_color1)=empty;
//*test_color2=empty;
//*sample_color1=empty;
//*sample_color2=empty;
extern int textHeight;
extern recomm_num;
extern good;
extern bad;
extern ans;

/*struct results{

    Graphics_Button dialog;

}*/

void updateMenuScreen(uint16_t new){

    // Disable touchInitialize_touch
    P2IE &= ~TOUCH_X_MINUS_PIN;
    P2IFG &= ~TOUCH_X_MINUS_PIN;

    // Draw new page
    if (new/10 != menu)
        switch(new/10){
            // Main menu
            case(0):
                drawMainMenu();
                break;
            // Settings
            case(1):
                drawSettings();
                break;
            // Between page
            case(2):
                drawInBetween();
                break;
            // Results page
            case(3):
                drawResults();
                break;
            default:
                break;
        }
    menu = new;

    // Re-enable touch
    //Initialize_touch();
    P2IE |= TOUCH_X_MINUS_PIN;
    P2IFG |= TOUCH_X_MINUS_PIN;
}
uint16_t getCurrMenuScreen(){
    return menu;
}

void eraseButton(Graphics_Button* button){
    Graphics_Button temp = *button;
    temp.fillColor = GRAPHICS_COLOR_BLACK;
    temp.borderColor = GRAPHICS_COLOR_BLACK;
    temp.textColor = GRAPHICS_COLOR_BLACK;
    // ------
    Graphics_drawButton(&g_sContext, &temp);
    //*button = temp;
    // ------
}

void initializeDemoButtons(void)
{
    // Initiliaze primitives Demo Button
    primitiveButton.xPosition = 20;
    primitiveButton.yPosition = 50;
    primitiveButton.borderWidth = 5;
    primitiveButton.selected = false;
    primitiveButton.imageWidth = Primitives_Button4BPP_UNCOMP.xSize;
    primitiveButton.imageHeight = Primitives_Button4BPP_UNCOMP.ySize;
    primitiveButton.borderColor = GRAPHICS_COLOR_WHITE;
    primitiveButton.selectedColor = GRAPHICS_COLOR_RED;
    primitiveButton.image = &Primitives_Button4BPP_UNCOMP;

    // Initiliaze images Demo Button
    imageButton.xPosition = 180;
    imageButton.yPosition = 50;
    imageButton.borderWidth = 5;
    imageButton.selected = false;
    imageButton.imageWidth = Primitives_Button4BPP_UNCOMP.xSize;
    imageButton.imageHeight = Primitives_Button4BPP_UNCOMP.ySize;
    imageButton.borderColor = GRAPHICS_COLOR_WHITE;
    imageButton.selectedColor = GRAPHICS_COLOR_RED;
    imageButton.image = &Images_Button4BPP_UNCOMP;

    yesButton.xMin = 80;
    yesButton.xMax = 150;
    yesButton.yMin = 80;
    yesButton.yMax = 120;
    yesButton.borderWidth = 1;
    yesButton.selected = false;
    yesButton.fillColor = GRAPHICS_COLOR_RED;
    yesButton.borderColor = GRAPHICS_COLOR_RED;
    yesButton.selectedColor = GRAPHICS_COLOR_BLACK;
    yesButton.textColor = GRAPHICS_COLOR_BLACK;
    yesButton.selectedTextColor = GRAPHICS_COLOR_RED;
    yesButton.textXPos = 100;
    yesButton.textYPos = 90;
    yesButton.text = "YES";
    yesButton.font = &g_sFontCm18;

    noButton.xMin = 180;
    noButton.xMax = 250;
    noButton.yMin = 80;
    noButton.yMax = 120;
    noButton.borderWidth = 1;
    noButton.selected = false;
    noButton.fillColor = GRAPHICS_COLOR_RED;
    noButton.borderColor = GRAPHICS_COLOR_RED;
    noButton.selectedColor = GRAPHICS_COLOR_BLACK;
    noButton.textColor = GRAPHICS_COLOR_BLACK;
    noButton.selectedTextColor = GRAPHICS_COLOR_RED;
    noButton.textXPos = 200;
    noButton.textYPos = 90;
    noButton.text = "NO";
    noButton.font = &g_sFontCm18;
}

void initializeButtons(void)
{
    // Default buttons, used as templates
    yesButton.xMin = 80;
    yesButton.xMax = 150;
    yesButton.yMin = 80;
    yesButton.yMax = midButtonYmax;
    yesButton.borderWidth = 5;
    yesButton.selected = false;
    yesButton.fillColor = GRAPHICS_COLOR_GRAY;
    yesButton.borderColor = GRAPHICS_COLOR_WHITE;
    yesButton.selectedColor = GRAPHICS_COLOR_BLACK;
    yesButton.textColor = GRAPHICS_COLOR_WHITE;
    yesButton.selectedTextColor = GRAPHICS_COLOR_RED;
    yesButton.textXPos = 100;
    yesButton.textYPos = 90;
    yesButton.text = "Settings";
    yesButton.font = &g_sFontCm18;

    noButton.xMin = 180;
    noButton.xMax = 250;
    noButton.yMin = 80;
    noButton.yMax = midButtonYmax;
    noButton.borderWidth = 5;
    noButton.selected = false;
    noButton.fillColor = GRAPHICS_COLOR_GRAY;
    noButton.borderColor = GRAPHICS_COLOR_WHITE;
    noButton.selectedColor = GRAPHICS_COLOR_BLACK;
    noButton.textColor = GRAPHICS_COLOR_WHITE;
    noButton.selectedTextColor = GRAPHICS_COLOR_RED;
    noButton.textXPos = 200;
    noButton.textYPos = 90;
    noButton.text = "Start";
    noButton.font = &g_sFontCm18;

    // Button to go into settings
    settingsButton.xMin = 70;
    settingsButton.xMax = 150;
    settingsButton.yMin = 80;
    settingsButton.yMax = midButtonYmax;
    settingsButton.borderWidth = 5;
    settingsButton.selected = false;
    settingsButton.fillColor = GRAPHICS_COLOR_GRAY;
    settingsButton.borderColor = GRAPHICS_COLOR_WHITE;
    settingsButton.selectedColor = GRAPHICS_COLOR_BLACK;
    settingsButton.textColor = GRAPHICS_COLOR_WHITE;
    settingsButton.selectedTextColor = GRAPHICS_COLOR_RED;
    settingsButton.textXPos = 80;
    settingsButton.textYPos = 90;
    settingsButton.text = "Settings";
    settingsButton.font = &g_sFontCm18;

    // Button to start touch calibration (Settings menu)
    calibrateButton.xMin = 60;
    calibrateButton.xMax = 160;
    calibrateButton.yMin = 80;
    calibrateButton.yMax = midButtonYmax;
    calibrateButton.borderWidth = 5;
    calibrateButton.selected = false;
    calibrateButton.fillColor = GRAPHICS_COLOR_GRAY;
    calibrateButton.borderColor = GRAPHICS_COLOR_WHITE;
    calibrateButton.selectedColor = GRAPHICS_COLOR_BLACK;
    calibrateButton.textColor = GRAPHICS_COLOR_WHITE;
    calibrateButton.selectedTextColor = GRAPHICS_COLOR_RED;
    calibrateButton.textXPos = 65;
    calibrateButton.textYPos = 90;
    calibrateButton.text = "Calib. Touch";
    calibrateButton.font = &g_sFontCm18;

    // Button to change set brightness (Settings menu)
    brightnessButton.xMin = 80;
    brightnessButton.xMax = 150;
    brightnessButton.yMin = 80;
    brightnessButton.yMax = midButtonYmax;
    brightnessButton.borderWidth = 5;
    brightnessButton.selected = false;
    brightnessButton.fillColor = GRAPHICS_COLOR_GRAY;
    brightnessButton.borderColor = GRAPHICS_COLOR_WHITE;
    brightnessButton.selectedColor = GRAPHICS_COLOR_BLACK;
    brightnessButton.textColor = GRAPHICS_COLOR_WHITE;
    brightnessButton.selectedTextColor = GRAPHICS_COLOR_RED;
    brightnessButton.textXPos = 100;
    brightnessButton.textYPos = 90;
    brightnessButton.text = "Adjust Brightness";
    brightnessButton.font = &g_sFontCm18;

    // Button that goes to main menu screen
    homeButton.xMin = 180;
    homeButton.xMax = 250;
    homeButton.yMin = 80;
    homeButton.yMax = midButtonYmax;
    homeButton.borderWidth = 5;
    homeButton.selected = false;
    homeButton.fillColor = GRAPHICS_COLOR_GRAY;
    homeButton.borderColor = GRAPHICS_COLOR_WHITE;
    homeButton.selectedColor = GRAPHICS_COLOR_BLACK;
    homeButton.textColor = GRAPHICS_COLOR_WHITE;
    homeButton.selectedTextColor = GRAPHICS_COLOR_RED;
    homeButton.textXPos = 200;
    homeButton.textYPos = 90;
    homeButton.text = "Home";
    homeButton.font = &g_sFontCm18;

    // Button to start/continue operations
    opButton.xMin = 180;
    opButton.xMax = 250;
    opButton.yMin = 80;
    opButton.yMax = midButtonYmax;
    opButton.borderWidth = 5;
    opButton.selected = false;
    opButton.fillColor = GRAPHICS_COLOR_GRAY;
    opButton.borderColor = GRAPHICS_COLOR_WHITE;
    opButton.selectedColor = GRAPHICS_COLOR_BLACK;
    opButton.textColor = GRAPHICS_COLOR_WHITE;
    opButton.selectedTextColor = GRAPHICS_COLOR_RED;
    opButton.textXPos = 200;
    opButton.textYPos = 90;
    opButton.text = "Start";
    opButton.font = &g_sFontCm18;

    // Button to put screen to sleep
    sleepButton.xMin = 240;
    sleepButton.xMax = 310;
    sleepButton.yMin = highButtonYmax - buttonHeight + 10;
    sleepButton.yMax = highButtonYmax;
    sleepButton.borderWidth = 4;
    sleepButton.selected = false;
    sleepButton.fillColor = GRAPHICS_COLOR_GRAY;
    sleepButton.borderColor = GRAPHICS_COLOR_WHITE;
    sleepButton.selectedColor = GRAPHICS_COLOR_BLACK;
    sleepButton.textColor = GRAPHICS_COLOR_WHITE;
    sleepButton.selectedTextColor = GRAPHICS_COLOR_RED;
    sleepButton.textXPos = 260;
    sleepButton.textYPos = 13;
    sleepButton.text = "Sleep";
    sleepButton.font = &g_sFontCm18;

    // Results dialog box
    results.xMin = (LCD_HORIZONTAL_MAX/2)-20;
    results.xMax = (LCD_HORIZONTAL_MAX/2)+60;
    results.yMin = (LCD_VERTICAL_MAX/2) - 15;
    results.yMax = (LCD_VERTICAL_MAX/2) + 15;
    results.borderWidth = 5;
    results.selected = false;
    results.fillColor = GRAPHICS_COLOR_GRAY;
    results.borderColor = GRAPHICS_COLOR_WHITE;
    results.textColor = GRAPHICS_COLOR_WHITE;
    results.text = "BLANK";
    results.textXPos = ((results.xMin + results.xMax)/2);
    results.textYPos = ((results.yMin + results.yMax)/2);
    results.font = &g_sFontCm18;
}

void putToSleep(void){

    // Disable touch interrupt and timer
    P2IFG &= ~TOUCH_X_MINUS_PIN;
    P2IE &= ~TOUCH_X_MINUS_PIN;
    P2IFG &= ~TOUCH_X_MINUS_PIN;
    TA1CCTL1 &= ~CCIFG;
    TA1CCTL1 &= ~CCIE;

    // Turn off screen
    GPIO_setOutputLowOnPin(LCD_PWM_PORT,
                                LCD_PWM_PIN);

    // Reset pins and re-activate interrupts for touch
    P2IE |= TOUCH_X_MINUS_PIN;
    P2IFG &= ~TOUCH_X_MINUS_PIN;
}

// Returns rounded results in milli scale (int)
int beer_lambert(double measGreen, double measBlue){

    measGreen = (double)measGreen;
    measBlue = (double)measBlue;

    /*if (ans == 1){
        return 0.1;
    }
    else if (ans == 2){
        return 3.5;
    }
    else if( ans == 3){
        return 1.0;
    }*/
    float multiplier = 0.9281f;

    float absorbGreen = log10f(measGreen);
    absorbGreen *= multiplier;
    float absorbBlue = log10f(measBlue);

    // Convert to integer to avoid floating-point issues on return
    // and factor up the integer for more accuracy
    // Factor up
    absorbGreen *= 1000;
    absorbBlue *= 1000;

//    calBlue = 1.7957*absorbGreen - 0.9825;
    return ((int)absorbBlue - (int)absorbGreen);
}

void run(int test){

    int color1;
    int color2;
    // Set delay timer
    setLaserTimer();
    setDetectorADC();

    // Turn on laser1
    P6OUT |= BIT0;
    P1OUT |= redLED;

    // Stall
    //--- Restart timer ---
    TA1CTL |= TACLR;
    //--- Clear flag ---
    TA1CCTL0 &= ~TAIFG;
    TA1CCTL0 &= ~CCIFG;
    //--- Enable interrupt for channel 0---
    TA1CCTL0 |= CCIE;
    //--- Disable interrupt for main timer AND touch pin
    TA1CTL &= ~TAIE;
    P2IE &= ~TOUCH_X_MINUS_PIN;
//    _low_power_mode_3();
    while((TA1CCTL0 & CCIFG) == 0){}
    TA1CCTL0 &= ~CCIFG;
    TA1CCTL0 &= ~CCIE;


    // Start timer for delay of ADC
    ADC12MEM2 =0;
    ADC12IFGR0 &= ~ADC12IFG2;
    ADC12CTL0 |= ADC12SC;
    // Wait for conversion to finish
    ///////////////////////////////////////////
     while((ADC12IFGR0 & ADC12IFG2) == 0); //
    ///////////////////////////////////////////

    volatile int color_temp = ADC12MEM2;
//    printToDisplay(color_temp);
    if(test){
        test_color1 = color_temp;
        color1 = test_color1;
    }
    else{
        sample_color1 = color_temp;
        color1 = sample_color1;
    }
    // Turn off laser1 AND timer interrupt
    P6OUT &= ~(BIT0);
    P1OUT &= ~redLED;

    // Stall
    //--- Restart timer ---
    TA1CTL |= TACLR;
    //--- Clear flag ---
    TA1CCTL0 &= ~TAIFG;
    TA1CCTL0 &= ~CCIFG;
    //--- Enable interrupt for channel 0---
    TA1CCTL0 |= CCIE;
    //--- Disable interrupt for main timer AND touch pin
    TA1CTL &= ~TAIE;
    P2IE &= ~TOUCH_X_MINUS_PIN;
    int temp = TA1CCR0;
    TA1CCR0 = TA1CCR1;
//    _low_power_mode_3();
    while((TA1CCTL0 & CCIFG) == 0){}
    TA1CCTL0 &= ~CCIFG;
    TA1CCR0 = temp;
    TA1CCTL0 &= ~CCIE;

    // Turn on laser2
    P6OUT |= BIT1;
    P1OUT |= redLED;

    // Stall
    //--- Restart timer ---
    TA1CTL |= TACLR;
    //--- Clear flag ---
    TA1CCTL0 &= ~TAIFG;
    //--- Enable interrupt for channel 0---
    TA1CCTL0 |= CCIE;
    //--- Disable interrupt for main timer AND touch pin
    TA1CTL &= ~TAIE;
    P2IE &= ~TOUCH_X_MINUS_PIN;
//    _low_power_mode_3();
    while((TA1CCTL0 & CCIFG) == 0){}
    TA1CCTL0 &= ~CCIFG;
    TA1CCTL0 &= ~CCIE;

    // Start timer for delay of ADC
    ADC12MEM2 =0;
    ADC12IFGR0 &= ~ADC12IFG2;
    ADC12CTL0 |= ADC12SC;
    // Wait for conversion to finish
    ///////////////////////////////////////////
     while((ADC12IFGR0 & ADC12IFG2) == 0); //
    ///////////////////////////////////////////
     color_temp = ADC12MEM2;
     if(test){
         test_color2 = color_temp;
         color2 = test_color2;
     }
     else{
         sample_color2 = color_temp;
         color2 = sample_color2;
     }
    // Turn off laser2
    P6OUT &= ~(BIT1);
    P1OUT &= ~redLED;

//    printToDisplay(color1);
//    printToDisplay_height(color2, (LCD_VERTICAL_MAX - 35));
    // Re-enable touch
    P2IE |= TOUCH_X_MINUS_PIN;

    // Reset touch delay
    setSleepTimer();
    setTouchADC();
}

void mainMenu(uint16_t x, uint16_t y){

    if(Graphics_isButtonSelected(&settingsButton,
                                      x,
                                      y))
    {
        Graphics_drawSelectedButton(&g_sContext,&settingsButton);
        Graphics_clearDisplay(&g_sContext);

        // Disable touch pins and interrupts before re-calibrating
        P2IE = 0;
        P2IFG = 0;
        P2IE |= BIT3;
        updateMenuScreen(10);
//        runPrimitivesDemo(x, y);
    }
    else if(Graphics_isButtonSelected(&opButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&opButton);

        // ---- THIS IS JUST FOR DEMO ----
        // Toggle fans
        P2OUT ^= (BIT1 | BIT2);
        P9OUT ^= greenLED;
        // -------------------------------

        // Clear un-needed buttons
        eraseButton(&opButton);
        eraseButton(&settingsButton);

        // Get rid of text from last screen
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawStringCentered(&g_sContext, "Welcome to DABS ^-^",
                                    AUTO_STRING_LENGTH,
                                    159,
                                    textHeight,
                                    TRANSPARENT_TEXT);

        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
            Graphics_drawStringCentered(&g_sContext, "Running sample. One moment...",
                                        AUTO_STRING_LENGTH,
                                        159,
                                        textHeight,
                                        TRANSPARENT_TEXT);


        // Run first reading
        run(1);


        updateMenuScreen(20);
        //runImagesDemo(x, y);
    }
    else if(Graphics_isButtonSelected(&sleepButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&sleepButton);

        putToSleep();

        Graphics_drawButton(&g_sContext,&sleepButton);

    }
}

void saveHome(){
   homeTemp = homeButton;
}

void saveOp(){
   opTemp = opButton;
}

void resetHome(){
    homeButton = homeTemp;
}

void resetOp(){
    opButton = opTemp;
}

// Menu number is 1
void settingsMenu(uint16_t x, uint16_t y){
    if(Graphics_isButtonSelected(&calibrateButton,
                                          x,
                                          y))
        {
            Graphics_drawSelectedButton(&g_sContext,&calibrateButton);
            P2IE = 0;
            P2IFG = 0;
            P2IE |= BIT3;
            Initialize_touch();
            Graphics_clearDisplay(&g_sContext);
            setCal(1);
            touch_calibrate();

            // Print that calibration is complete
            // Leave on setting screen in case it needs to be re-calibrated
            // ...
        }
    else if(Graphics_isButtonSelected(&homeButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&homeButton);
        updateMenuScreen(0);
    }
}

// Menu number is 2
void continuingMenu(uint16_t x, uint16_t y){
    if(Graphics_isButtonSelected(&homeButton,
                                      x,
                                      y))
    {
        Graphics_drawSelectedButton(&g_sContext,&homeButton);
        resetHome();
        resetOp();
        updateMenuScreen(0);
    }
    else if(Graphics_isButtonSelected(&opButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&opButton);

        // Clear un-needed buttons
        eraseButton(&opButton);
        eraseButton(&homeButton);

        // ---- THIS IS JUST FOR DEMO ----
        // Toggle fans
        P2OUT ^= (BIT1 | BIT2);
        P9OUT ^= greenLED;

        // Get rid of text from last screen
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawStringCentered(&g_sContext, "Would you like to continue?",
                                    AUTO_STRING_LENGTH,
                                    159,
                                    textHeight,
                                    TRANSPARENT_TEXT);

        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
        Graphics_drawStringCentered(&g_sContext, "Running sample. One moment...",
                                    AUTO_STRING_LENGTH,
                                    159,
                                    textHeight,
                                    TRANSPARENT_TEXT);
        // Run second reading
        run(0);

        float final_result = (float)beer_lambert( (test_color1/(sample_color1)), (test_color2/(sample_color2)) );

        // Get rid of text from last screen
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawStringCentered(&g_sContext, "Running sample. One moment...",
                                    AUTO_STRING_LENGTH,
                                    159,
                                    textHeight,
                                    TRANSPARENT_TEXT);
        finalize(final_result);

        resetHome();
        resetOp();
        updateMenuScreen(30);

//        printToDisplay(test_color1);
//        printToDisplay_height(sample_color1, (LCD_VERTICAL_MAX - 35));
    }
    else if(Graphics_isButtonSelected(&sleepButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&sleepButton);

        putToSleep();

        Graphics_drawButton(&g_sContext,&sleepButton);

    }

}

int finalize(float result){

    // recomm_num => (Start, length); 0 => Still blank
//    if(ans <= good){
    if (ans == 1){
        recomm_num= 12;
        results.borderColor = GRAPHICS_COLOR_WHITE;
        results.textColor = GRAPHICS_COLOR_WHITE;
        results.fillColor = GRAPHICS_COLOR_GREEN;
        results.text = "SAFE";
        ans = 2;
    }
//    else if (good < ans && ans < bad){
    else if( ans == 2){
        recomm_num = 31;
        results.borderColor = GRAPHICS_COLOR_WHITE;
        results.textColor = GRAPHICS_COLOR_BLACK;
        results.fillColor = GRAPHICS_COLOR_YELLOW;
        results.text = "ELEVATED";
        ans = 3;
    }
    else{
        recomm_num = 42;
        results.borderColor = GRAPHICS_COLOR_WHITE;
        results.textColor = GRAPHICS_COLOR_WHITE;
        results.fillColor = GRAPHICS_COLOR_RED;
        results.text = "DANGEROUS";
        ans = 1;
    }
}

void resultsMenu(uint16_t x, uint16_t y){
    if(Graphics_isButtonSelected(&homeButton,
                                      x,
                                      y))
    {
        Graphics_drawSelectedButton(&g_sContext,&homeButton);
        resetHome();
        updateMenuScreen(0);
    }
    else if(Graphics_isButtonSelected(&sleepButton,
                                           x,
                                           y))
    {
        Graphics_drawSelectedButton(&g_sContext,&sleepButton);

        putToSleep();

        Graphics_drawButton(&g_sContext,&sleepButton);

    }
}

unsigned int find_OOM(unsigned int n){

    unsigned int k = 10000;

    do{
        if(n >= k)
            break;
    }while( (k /= 10) > 1);

    return k;
}

void printToDisplay_height(double n, unsigned int height){
    float num = (float) n;
//    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    unsigned char ch[1], i;
    // Getting the upper numbers and the decimals to 3 places
    unsigned int upper = 0;
    unsigned int lower;

    unsigned char upper_string[4];           // 4 digits plus null terminator
    unsigned char lower_string[4];           // 4 digits plus null terminator

    int mag;

    if(num > 9999 ){
        Graphics_drawStringCentered(&g_sContext, "> 9999",
                    AUTO_STRING_LENGTH,
                    (LCD_HORIZONTAL_MAX/2),
                    height,
                    TRANSPARENT_TEXT);
        return;
    }
    // Print integral/whole numbers if they exist
    if(num >= 1){
        upper = (int)num;
        mag = find_OOM(upper);
        i = 0;
        do{
            upper_string[i] = '0' + (upper/mag)%10;

            i++;
        }while((mag /= 10) > 0);

        *(upper_string+i) = (char)'\0';

        Graphics_drawStringCentered(&g_sContext, upper_string,
            AUTO_STRING_LENGTH,
            (LCD_HORIZONTAL_MAX/2) - (LCD_HORIZONTAL_MAX/6),
            height,
            TRANSPARENT_TEXT);
    }
    else{
        ch[0] = '0';
            Graphics_drawStringCentered(&g_sContext, ch,
                AUTO_STRING_LENGTH,
                (LCD_HORIZONTAL_MAX/2) - 15,
                height,
                TRANSPARENT_TEXT);
    }

    ch[0] = '.';
    Graphics_drawStringCentered(&g_sContext, ch,
        AUTO_STRING_LENGTH,
        (LCD_HORIZONTAL_MAX/2),
        height,
        TRANSPARENT_TEXT);

    // For decimals
    num -= (int)num;
    num = (num*10000);
    lower  = ((int) num);
    mag = find_OOM(lower);

    i = 0;
    do{
        lower_string[i] = '0' + (lower/mag)%10;

        i++;
    }while((mag /= 10) > 0);
    *(lower_string+i) = (char)'\0';

    Graphics_drawStringCentered(&g_sContext, lower_string,
        AUTO_STRING_LENGTH,
        (LCD_HORIZONTAL_MAX/2) + (LCD_HORIZONTAL_MAX/6),
        height,
        TRANSPARENT_TEXT);
    return;
}

void printToDisplay(const float num){

    printToDisplay_height(num, (LCD_VERTICAL_MAX - 20));

}
