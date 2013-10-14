/*
 * motortest.c
 *
 *  Created on: Oct 13, 2013
 *      Author: xander
 */

#include <unistd.h>
#include <stdio.h>
#include <unistd.h>
#include "lms2012.h"
#include "lms_output.h"
#include "lms_buttons.h"
#include "lms_lcd.h"
#include "robotclogo.xbm"

char buttonState[BUTTONS];

void initAll()
{
  outputInit();
  lcdInit();
  buttonsInit();

  // Set the LED to red
  ledSet(LED_RED);

  // Configure motor port A for standard EV3 motor
  outputSetType(0, CONN_OUTPUT_INTELLIGENT);
  // Make sure all motors are stopped
  outputStop(0xFF, 0);
  // clear the tacho
  outputClrCount(0x01);
}


int main (int argc, char **argv)
{
  int16_t  motorSpeed = 0;
  int32_t tachoCount = 0;
  int16_t oldMotorSpeed = 0;
  char textBuff[64];

  initAll();

  // Our main loop
  while (1 == 1)
  {
    // Read the buttons
    buttonsReadDebounced(buttonState);

    // Read the current tacho count
    outputGetCount(0, &tachoCount);

    // Increment the motor speed if right button is pressed
    // Decrement the motor speed if the left button is pressed
    // Stop the motor if the middle button is pressed
    if (buttonState[RIGHT_BUTTON - 1])
      motorSpeed += 10;
    else if (buttonState[LEFT_BUTTON - 1])
      motorSpeed -= 10;
    else if (buttonState[ENTER_BUTTON -1])
      motorSpeed = 0;

    // Make sure we stay inside the motor speed range
    if (motorSpeed > 100)
      motorSpeed = 100;
    if (motorSpeed < -100)
      motorSpeed = -100;

    // clear the screen buffer
    lcdClearScreen();

    // Pretty up the output
    sprintf(&textBuff[0], "Speed: %d", motorSpeed);
    lcdDrawStringSmall(&textBuff[0], 75, 0);

    sprintf(&textBuff[0], "Tacho: %d", tachoCount);
    lcdDrawStringSmall(&textBuff[0], 75, 10);

    sprintf(&textBuff[0], "LEFT :speed--");
    lcdDrawStringSmall(&textBuff[0], 75, 30);

    sprintf(&textBuff[0], "RIGHT:speed++");
    lcdDrawStringSmall(&textBuff[0], 75, 40);

    sprintf(&textBuff[0], "ENTER:stop");
    lcdDrawStringSmall(&textBuff[0], 75, 50);


    // Draw the ROBOTC logo
    lcdBitBltLCD(logoleft_bits, logoleft_width, logoleft_height, 0, 0, 0, 0, logoleft_width, logoleft_height, RASTER_OP_COPY);

    // Only update if there's been a change
    if (oldMotorSpeed != motorSpeed)
    {
      if (motorSpeed != 0)
      {
        outputSpeed(0x01, motorSpeed);
        outputStart(0x01);
      }
      else
      {
        outputStop(0x01, 0);
        outputClrCount(0x01);
      }

      oldMotorSpeed = motorSpeed;

      if (motorSpeed < -66)
        ledSet(LED_ORANGE_PULSE);
      else if (motorSpeed < -33)
        ledSet(LED_ORANGE_FLASH);
      else if (motorSpeed < 0)
        ledSet(LED_ORANGE);
      else if (motorSpeed == 0)
        ledSet(LED_RED);
      else if (motorSpeed > 66)
        ledSet(LED_ORANGE_PULSE);
      else if (motorSpeed > 33)
        ledSet(LED_ORANGE_FLASH);
      else if (motorSpeed > 0)
        ledSet(LED_ORANGE);
    }

    lcdUpdateScreen();

    // Sleep 20 ms
    usleep(20*1000);

  }
}