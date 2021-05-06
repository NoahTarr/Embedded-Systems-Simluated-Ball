//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - I2C 
// Application Overview - The objective of this application is act as an I2C 
//                        diagnostic tool. The demo application is a generic 
//                        implementation that allows the user to communicate 
//                        with any I2C device over the lines. 
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup i2c_demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

// Common interface includes
#include "uart_if.h"
#include "i2c_if.h"

#include "pin_mux_config.h"
#include "Adafruit_SSD1351.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME                "I2C Demo"
#define UART_PRINT              Report
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// OLED Dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define BALL_RADIUS 4
#define TIME_UNIT_BTWN_PX 1000
int accelerometerX = 0x0;
int accelerometerY = 0x0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************

//*****************************************************************************
//
//! Display a prompt for the user to enter command
//!
//! \param  none
//!
//! \return none
//! 
//*****************************************************************************
void 
DisplayPrompt()
{
    UART_PRINT("\n\rcmd#");
}

//*****************************************************************************
//
//! Display the usage of the I2C commands supported
//!
//! \param  none
//!
//! \return none
//! 
//*****************************************************************************
void 
DisplayUsage()
{
    UART_PRINT("Command Usage \n\r");
    UART_PRINT("------------- \n\r");
    UART_PRINT("write <dev_addr> <wrlen> <<byte0> [<byte1> ... ]> <stop>\n\r");
    UART_PRINT("\t - Write data to the specified i2c device\n\r");
    UART_PRINT("read  <dev_addr> <rdlen> \n\r\t - Read data frpm the specified "
                "i2c device\n\r");
    UART_PRINT("writereg <dev_addr> <reg_offset> <wrlen> <<byte0> [<byte1> ... "
                "]> \n\r");
    UART_PRINT("\t - Write data to the specified register of the i2c device\n\r");
    UART_PRINT("readreg <dev_addr> <reg_offset> <rdlen> \n\r");
    UART_PRINT("\t - Read data from the specified register of the i2c device\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("Parameters \n\r");
    UART_PRINT("---------- \n\r");
    UART_PRINT("dev_addr - slave address of the i2c device, a hex value "
                "preceeded by '0x'\n\r");
    UART_PRINT("reg_offset - register address in the i2c device, a hex value "
                "preceeded by '0x'\n\r");
    UART_PRINT("wrlen - number of bytes to be written, a decimal value \n\r");
    UART_PRINT("rdlen - number of bytes to be read, a decimal value \n\r");
    UART_PRINT("bytex - value of the data to be written, a hex value preceeded "
                "by '0x'\n\r");
    UART_PRINT("stop - number of stop bits, 0 or 1\n\r");
    UART_PRINT("--------------------------------------------------------------"
                "--------------- \n\r\n\r");

}

//*****************************************************************************
//
//! Display the buffer contents over I2C
//!
//! \param  pucDataBuf is the pointer to the data store to be displayed
//! \param  ucLen is the length of the data to be displayed
//!
//! \return none
//! 
//*****************************************************************************
void 
DisplayBuffer(unsigned char *pucDataBuf, unsigned char ucLen)
{
    unsigned char ucBufIndx = 0;
    UART_PRINT("Read contents");
    UART_PRINT("\n\r");
    while(ucBufIndx < ucLen)
    {
        UART_PRINT(" 0x%x, ", pucDataBuf[ucBufIndx]);
        ucBufIndx++;
        if((ucBufIndx % 8) == 0)
        {
            UART_PRINT("\n\r");
        }
    }
    UART_PRINT("\n\r");
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//****************************************************************************
//
//! Parses the read command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the user command parameters
//! 
//! This function  
//!    1. Parses the read command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
//unsigned char*
int
ProcessReadCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));
    
    //
    // Read the specified length of data
    //
    iRetVal = I2C_IF_Read(ucDevAddr, aucDataBuf, ucLen);

    if(iRetVal == SUCCESS)
    {
        UART_PRINT("I2C Read complete\n\r");
        
        //
        // Display the buffer over UART on successful write
        //
        DisplayBuffer(aucDataBuf, ucLen);
    }
    else
    {
        UART_PRINT("I2C Read failed\n\r");
        return FAILURE;
//        aucDataBuf[0] = '\0';
    }

    return SUCCESS;
//    return aucDataBuf;
}

//****************************************************************************
//
//! Parses the readreg command parameters and invokes the I2C APIs
//! i2c readreg 0x<dev_addr> 0x<reg_offset> <rdlen>
//!
//! \param pcInpString pointer to the readreg command parameters
//! 
//! This function  
//!    1. Parses the readreg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!    Next was added for EEC172 Lab 3
//!    3. Updates global accelerometer values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
#define ACCELX 0x03
#define ACCELY 0x05
void
updateAccelerometerAxis(char value, unsigned char ACCEL_AXIS)
{
    //Subtract 255 so flat is 0 and angled is either -64 or +64
    if (ACCEL_AXIS == ACCELX)
    {
        if (value < 128) accelerometerX = value;
        else accelerometerX = value - 255;
    }
    else if (ACCEL_AXIS == ACCELY)
    {
        if (value < 128) accelerometerY = value;
        else accelerometerY = value - 255;
    }
}


int
ProcessReadRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    char *pcErrPtr;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the register offset address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRdLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));
    
    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

//    UART_PRINT("I2C Read From address complete\n\r");
    
    //
    // Display the buffer over UART on successful readreg
    //
//    DisplayBuffer(aucRdDataBuf, ucRdLen);

    updateAccelerometerAxis(aucRdDataBuf[0],ucRegOffset);

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the writereg command parameters and invokes the I2C APIs
//! i2c writereg 0x<dev_addr> 0x<reg_offset> <wrlen> <0x<byte0> [0x<byte1> ...]>
//!
//! \param pcInpString pointer to the readreg command parameters
//! 
//! This function  
//!    1. Parses the writereg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucWrLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iLoopCnt = 0;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    
    //
    // Get the register offset to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    aucDataBuf[iLoopCnt] = ucRegOffset;
    iLoopCnt++;
    
    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucWrLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucWrLen > sizeof(aucDataBuf));
   
    //
    // Get the bytes to be written
    //
    for(; iLoopCnt < ucWrLen + 1; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] = 
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }
    //
    // Write the data values.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&aucDataBuf[0],ucWrLen+1,1));

    UART_PRINT("I2C Write To address complete\n\r");

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the write command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the write command parameters
//! 
//! This function  
//!    1. Parses the write command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucStopBit, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal, iLoopCnt;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    
    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    for(iLoopCnt = 0; iLoopCnt < ucLen; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] = 
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }
    
    //
    // Get the stop bit
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucStopBit = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    
    //
    // Write the data to the specified address
    //
    iRetVal = I2C_IF_Write(ucDevAddr, aucDataBuf, ucLen, ucStopBit);
    if(iRetVal == SUCCESS)
    {
        UART_PRINT("I2C Write complete\n\r");
    }
    else
    {
        UART_PRINT("I2C Write failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the user input command and invokes the I2C APIs
//!
//! \param pcCmdBuffer pointer to the user command
//! 
//! This function  
//!    1. Parses the user command.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ParseNProcessCmd(char *pcCmdBuffer)
{
    char *pcInpString;
    int iRetVal = FAILURE;

    pcInpString = strtok(pcCmdBuffer, " \n\r");
    if(pcInpString != NULL)

    {
           
        if(!strcmp(pcInpString, "read"))
        {
            //
            // Invoke the read command handler
            //
            iRetVal = ProcessReadCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "readreg"))
        {
            //
            // Invoke the readreg command handler
            //
            iRetVal = ProcessReadRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "writereg"))
        {
            //
            // Invoke the writereg command handler
            //
            iRetVal = ProcessWriteRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "write"))
        {
            //
            // Invoke the write command handler
            //
            iRetVal = ProcessWriteCommand(pcInpString);
        }
        else
        {
            UART_PRINT("Unsupported command\n\r");
            return FAILURE;
        }
    }

    return iRetVal;
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function handling the I2C example
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************
int
calcVelocity(int meterVal)
{
    int velocity = 1;
    if (meterVal < 0) velocity = -1;
    meterVal = abs(meterVal);

    if (meterVal < 7) velocity *= 0;
    else if (meterVal < 13) velocity *= 2;
    else if (meterVal < 21) velocity *= 4;
    else if (meterVal < 29) velocity *= 7;
    else if (meterVal < 37) velocity *= 11;
    else if (meterVal < 45) velocity *= 16;
    else if (meterVal < 55) velocity *= 23;
    else velocity *= 15;
    return velocity;
}

int
calcAcceleration(int meterVal)
{
    int acceleration = 1;
    if (meterVal < 0) acceleration = -1;
    meterVal = abs(meterVal);

    if (meterVal > 50) acceleration *= 2;
    else if (meterVal > 7) acceleration *= 1;
    else acceleration = 0;

    return acceleration;
}

int min(int a, int b)
{
    if (a <= b)
        return a;
    return b;
}

int max(int a, int b)
{
    if (a >= b)
        return a;
    return b;
}



void main()
{
    BoardInit();
    PinMuxConfig();
    InitTerm();
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    
    // Display the banner followed by the usage description
    DisplayBanner(APP_NAME);
    DisplayUsage();

    // Init SPI
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVELOW | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);
    Adafruit_Init();
    fillScreen(0x00);


    char accelXCmd[512];
    char accelYCmd[512];

    int xPos, yPos;
    unsigned int xPos0, yPos0;
    float xVel, yVel;
    float xAccl, yAccl;
    xPos = 64;
    yPos = 64;
    xVel = 0;
    yVel = 0;
    xAccl = 0;
    yAccl = 0;
    drawCircle(xPos, yPos, BALL_RADIUS, 0xffff);

    int maxPixelVel;
    float maxPixelAccl;
    maxPixelVel = 5;
    maxPixelAccl = 2;
    while (1)
    {
        //Get accelerometer values
        strcpy(accelXCmd, "readreg 0x18 0x3 1");
        strcpy(accelYCmd, "readreg 0x18 0x5 1");
        ParseNProcessCmd(accelXCmd);
        ParseNProcessCmd(accelYCmd);
        UART_PRINT("X: %i\n\r", accelerometerX);
        UART_PRINT("Y: %i\n\r", accelerometerY);


        xPos0 = xPos;
        yPos0 = yPos;


        //Calculate X position, velocity, and acceleration
        xAccl += calcAcceleration(accelerometerX);
        if (xAccl > maxPixelAccl) xAccl = maxPixelAccl;
        else if (xAccl < -maxPixelAccl) xAccl = -maxPixelAccl;

        xPos = xPos + xVel + (int)xAccl/2;
        if (xPos >= 123)
        {
            xPos = 123;
            xAccl = -maxPixelAccl;
        }
        else if (xPos <= 5)
        {
            xPos = 5;
            xAccl = maxPixelAccl;
        }

        xVel += xAccl;
        if (xVel > maxPixelVel) xVel = maxPixelVel;
        else if (xVel < -maxPixelVel) xVel = -maxPixelVel;



        //Calculate Y position, velocity, and acceleration
        yAccl += calcAcceleration(accelerometerY);
        if (yAccl > maxPixelAccl) yAccl = maxPixelAccl;
        else if (yAccl < -maxPixelAccl) yAccl = -maxPixelAccl;

        yPos = yPos + yVel + (int)yAccl/2;
        if (yPos >= 123)
        {
            yPos = 123;
            yAccl = -maxPixelAccl;
        }
        else if (yPos <= 5)
        {
            yPos = 5;
            yAccl = maxPixelAccl;
        }

        yVel += yAccl;
        if (yVel > maxPixelVel) yVel = maxPixelVel;
        else if (yVel < -maxPixelVel) yVel = -maxPixelVel;



        //Fix values
        xVel -= xVel/4;
        yVel -= yVel/4;
        if ((xVel < 0 && xVel > -0.5) || (xVel > 0 && xVel < 0.5))
            xVel = 0;
        if ((yVel < 0 && yVel > -0.5) || (yVel > 0 && yVel < 0.5))
            yVel = 0;

        xAccl -= xAccl/4;
        yAccl -= yAccl/4;
        if ((xAccl < 0 && xAccl > -0.5) || (xAccl > 0 && xAccl < 0.5))
            xAccl = 0;
        if ((yAccl < 0 && yAccl > -0.5) || (yAccl > 0 && yAccl < 0.5))
            yAccl = 0;

        if (xPos0 == xPos && yPos0 == yPos)
            continue;

        drawCircle(yPos0, xPos0, BALL_RADIUS, 0x0000);
        drawCircle(yPos, xPos, BALL_RADIUS, 0xffff);
        UtilsDelay(10);
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************


