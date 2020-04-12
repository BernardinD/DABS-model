/*
 * menu.h
 *
 *  Created on: Dec 26, 2019
 *      Author: deziu
 */

#ifndef MENUS_H_
#define MENUS_H_


#include <msp430fr6989.h>
#include "driverlib.h"
#include "kitronix320x240x16_ssd2119_spi.h"
#include "HAL_MSP_EXP432P401R_KITRONIX320X240_SSD2119_SPI.h"
#include "grlib.h"
#include "button.h"
#include "imageButton.h"
#include "Images/images.h"

Graphics_ImageButton primitiveButton;
Graphics_ImageButton imageButton;
Graphics_Button yesButton;
Graphics_Button noButton;

Graphics_Button settingsButton;
Graphics_Button calibrateButton;
Graphics_Button brightnessButton;
Graphics_Button homeButton;
Graphics_Button opButton;
Graphics_Button sleepButton;
Graphics_Button results;

void initializeDemoButtons(void);
void initializeButtons(void);
void runPrimitivesDemo(uint16_t, uint16_t);
void runImagesDemo(uint16_t, uint16_t);
void drawRestarDemo(uint16_t, uint16_t);
void updateMenuScreen(uint16_t);
uint16_t getCurrMenuScreen();

//uint16_t menu = 1000;


#endif /* MENUS_H_ */
