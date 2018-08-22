/**
 *  \file   main_test.c
 *
 *  \brief  Example application main file. This application will read the data
 *          from eeprom and compares it with the known data.
 *
 */

/*
 * Copyright (C) 2014 - 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdio.h>
#include <string.h>

/* TI-RTOS Header files */
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include "I2C_log.h"
#include "I2C_board.h"

#if defined (SOC_AM335X) || defined (SOC_AM437x)
/* EEPROM data -Board specific */
extern char eepromData[I2C_EEPROM_RX_LENGTH];
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

#define I2C_TRANSACTION_TIMEOUT         (10000U)


/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Data compare function */
bool CompareData(char *expData, char *rxData, unsigned int length);

#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_AM571x) || defined (SOC_AM572x) || defined (SOC_AM574x)
/* Probe and runtime bus frequnecy configuration test */
static bool I2C_Probe_BusFrequency_test(I2C_Handle handle);
static bool I2C_timeout_test(I2C_Handle handle);
#endif
/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/*The accelerometer and gyroscope data are 16 bits wide and so data from each axis uses two registers.*/

char deviceADDR  = 0x68;        /* Chip Adress. */
char PWR_MGMT_1  = 0x6B;        /* To wake up the MPU6050. */
char ACCEL_X_OUT_H = 0x3B;
char ACCEL_X_OUT_L = 0x3C;
char ACCEL_Y_OUT_H = 0x3D;
char ACCEL_Y_OUT_L = 0x3E;
char ACCEL_Z_OUT_H = 0x3F;
char ACCEL_Z_OUT_L = 0x40;
char GYRO_X_OUT_H = 0x43;
char GYRO_X_OUT_L = 0x44;
char GYRO_Y_OUT_H = 0x45;
char GYRO_Y_OUT_L = 0x46;
char GYRO_Z_OUT_H = 0x47;
char GYRO_Z_OUT_L = 0x48;

/*
 *  ======== Board_initI2C ========
 */
bool Board_initI2C(void)
{
    Board_initCfg boardCfg;
    Board_STATUS  boardStatus;
#if defined (idkAM571x)
    Board_IDInfo  id;
#endif
    I2C_HwAttrs   i2c_cfg;
#if defined (evmK2G)
    Board_SoCInfo socInfo;
#endif

    /* Get the default I2C init configurations */
    I2C_socGetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

    /* Modify the default I2C configurations if necessary */

    /* Set the default I2C init configurations */
    I2C_socSetInitCfg(I2C_EEPROM_INSTANCE, &i2c_cfg);

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    boardStatus = Board_init(boardCfg);
    if (boardStatus != BOARD_SOK)
    {
        return (false);
    }


    return (true);
}
//
//void set(char reg1, char reg2, char valor){
//
//}

void AppDelay(int x);


int16_t set(char address, char reg, char value, I2C_Transaction i2cTransaction, I2C_Handle handle){


    char txBuf[2];
    char rxBuf;

    txBuf[0] = reg;
    txBuf[1] = value;

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf = txBuf;
    i2cTransaction.writeCount = sizeof(txBuf);
    i2cTransaction.readBuf = &rxBuf;
    i2cTransaction.readCount = sizeof(rxBuf);
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;

    return I2C_transfer(handle, &i2cTransaction);

}

int8_t get(char address, char reg, I2C_Transaction i2cTransaction, I2C_Handle handle){

    char txBuf;
    char rxBuf;

    txBuf = reg;

    i2cTransaction.slaveAddress = address;
    i2cTransaction.writeBuf = &txBuf;
    i2cTransaction.writeCount = sizeof(txBuf);
    i2cTransaction.readBuf = &rxBuf;
    i2cTransaction.readCount = sizeof(rxBuf);
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;

    I2C_transfer(handle, &i2cTransaction);
    return rxBuf;

}

/*
 *  ======== test function ========
 */
void i2c_test(UArg arg0, UArg arg1){


    I2C_Transaction i2cTransaction;
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_init();
    int8_t rxBuf;


    I2C_Params_init(&i2cParams);
    handle = I2C_open(1, &i2cParams);

    set(deviceADDR, PWR_MGMT_1, 0, i2cTransaction, handle);

    uint16_t gyroX, gyroY, gyroZ;

    while(1){
        rxBuf = get(deviceADDR, GYRO_X_OUT_H, i2cTransaction, handle);
        gyroX = rxBuf << 8;
        rxBuf = get(deviceADDR, GYRO_X_OUT_L, i2cTransaction, handle);
        gyroX += rxBuf;
        UART_printf("Giro no eixo X: %x | ", gyroX);

        rxBuf = get(deviceADDR, GYRO_Y_OUT_H, i2cTransaction, handle);
        gyroY = rxBuf << 8;
        rxBuf = get(deviceADDR, GYRO_Y_OUT_L, i2cTransaction, handle);
        gyroY += rxBuf;
        UART_printf("Giro no eixo Y: %x | ", gyroY);

        rxBuf = get(deviceADDR, GYRO_Z_OUT_H, i2cTransaction, handle);
        gyroZ = rxBuf << 8;
        rxBuf = get(deviceADDR, GYRO_Z_OUT_L, i2cTransaction, handle);
        gyroZ += rxBuf;
        UART_printf("Giro no eixo Z: %x | ", gyroZ);
        UART_printf("\n");
        AppDelay(2);
    }



    I2C_close(handle);

    while (1) {

    }
}

/*
 *  ======== main ========
 */
int main(void){


    if (Board_initI2C() == false)
    {
        return (0);
    }

    UART_printf("Cheguei aqui, carais! \n");

#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_OMAPL137)
    Task_Handle task;
    Error_Block eb;

    Error_init(&eb);

    task = Task_create(i2c_test, NULL, &eb);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }
#endif

    /* Start BIOS */
    BIOS_start();
    return (0);
}

/*
 *  ======== CompareData ========
 */
bool CompareData(char *expData, char *rxData, unsigned int length)
{
    uint32_t idx = 0;
    uint32_t match = 1;
    bool retVal = false;

    for(idx = 0; ((idx < length) && (match != 0)); idx++)
    {
        if(*expData != *rxData) match = 0;
        expData++;
        rxData++;
    }

    if(match == 1) retVal = true;

    return retVal;
}


#if defined (SOC_AM335X) || defined (SOC_AM437x) || defined (SOC_AM571x) || defined (SOC_AM572x) || defined (SOC_AM574x)
static bool I2C_Probe_BusFrequency_test(I2C_Handle handle)
{
    uint32_t busFrequency;
    bool status = false;
    int16_t transferStatus;
    I2C_Transaction i2cTransaction;
    uint32_t slaveAddress;
    int32_t controlStatus;
    char txBuf[I2C_EEPROM_TEST_LENGTH + I2C_EEPROM_ADDR_SIZE] = {0x00, };
    char rxBuf[I2C_EEPROM_TEST_LENGTH];
    uint32_t delayValue;

    /* Set the I2C EEPROM write/read address */
    txBuf[0] = (I2C_EEPROM_TEST_ADDR >> 8) & 0xff; /* EEPROM memory high address byte */
    txBuf[1] = I2C_EEPROM_TEST_ADDR & 0xff;        /* EEPROM memory low address byte */


    /* Test Runtime Configuration of Bus Frequency */

    /* Test runtime configuration of 400 kHz */
    busFrequency = I2C_400kHz;
    I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

    memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
    i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
    i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
    i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
    transferStatus = I2C_transfer(handle, &i2cTransaction);

    if(I2C_STS_SUCCESS != transferStatus)
    {
        I2C_log("\n I2C Test: Dynamic configuration of bus Freq failed. \n");
    }

    status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);

    if(true == status)
    {
        /* Test runtime configuration of 100 kHz */
        busFrequency = I2C_100kHz;
        I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

        memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
        I2C_transactionInit(&i2cTransaction);
        i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
        i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
        i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
        i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
        i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
        i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
        transferStatus = I2C_transfer(handle, &i2cTransaction);

        if(I2C_STS_SUCCESS != transferStatus)
        {
            I2C_log("\n I2C Test: Dynamic configuration of bus Freq failed. \n");
        }

        status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);
    }


    /* Test Probe functionality */

    if(true == status)
    {
        /* Probe test with valid slave address */
        slaveAddress = I2C_EEPROM_ADDR;
        controlStatus = I2C_control(handle, I2C_CMD_PROBE, &slaveAddress);

        if(I2C_STATUS_SUCCESS == controlStatus)
        {
            status = true;
        }
        else
        {
            status = false;
            I2C_log("\n I2C Test: Probe test failed. \n");
        }
    }

    if(true == status)
    {
        /* Probe test with invalid slave address */
        slaveAddress = 0x70U;
        controlStatus = I2C_control(handle, I2C_CMD_PROBE, &slaveAddress);

        if(I2C_STATUS_ERROR == controlStatus)
        {
            status = true;
        }
        else
        {
            status = false;
            I2C_log("\n I2C Test: Probe test failed. \n");
        }
    }

    if(true == status)
    {
        /* Test bus recovery functionality */
        delayValue = 2000U;
        controlStatus = I2C_control(handle, I2C_CMD_RECOVER_BUS, &delayValue);

        if(I2C_STATUS_SUCCESS == controlStatus)
        {
            memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
            I2C_transactionInit(&i2cTransaction);
            i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
            i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
            i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
            i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
            i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
            i2cTransaction.timeout   = I2C_TRANSACTION_TIMEOUT;
            transferStatus = I2C_transfer(handle, &i2cTransaction);

            if(I2C_STS_SUCCESS != transferStatus)
            {
                I2C_log("\n I2C Test: Bus recovery test failed. \n");
            }

            status = CompareData(&eepromData[0], &rxBuf[0], I2C_EEPROM_TEST_LENGTH);
        }
        else
        {
            status = false;
        }
    }

    return status;
}

static bool I2C_timeout_test(I2C_Handle handle)
{
    uint32_t busFrequency;
    bool status = false;
    int16_t transferStatus;
    I2C_Transaction i2cTransaction;
    char txBuf[I2C_EEPROM_TEST_LENGTH + I2C_EEPROM_ADDR_SIZE] = {0x00, };
    char rxBuf[I2C_EEPROM_TEST_LENGTH];

    /* Set the I2C EEPROM write/read address */
    txBuf[0] = (I2C_EEPROM_TEST_ADDR >> 8) & 0xff; /* EEPROM memory high address byte */
    txBuf[1] = I2C_EEPROM_TEST_ADDR & 0xff;        /* EEPROM memory low address byte */


    /* Test Runtime Configuration of Bus Frequency */

    /* Test runtime configuration of 400 kHz */
    busFrequency = I2C_100kHz;
    I2C_control(handle, I2C_CMD_SET_BUS_FREQUENCY, &busFrequency);

    memset(rxBuf, 0, I2C_EEPROM_TEST_LENGTH);
    I2C_transactionInit(&i2cTransaction);
    i2cTransaction.slaveAddress = I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = I2C_EEPROM_ADDR_SIZE;
    i2cTransaction.readBuf = (uint8_t *)&rxBuf[0];
    i2cTransaction.readCount = I2C_EEPROM_TEST_LENGTH;
    i2cTransaction.timeout   = 1;
    transferStatus = I2C_transfer(handle, &i2cTransaction);

    if(I2C_STS_ERR_TIMEOUT == transferStatus)
    {
        I2C_log("\n I2C Test: timeout test passed. \n");
        status = true;
    }
    return status;
}

/*
 *  ======== AppDelay ========
 */
void AppDelay(int x)
{
    if(x == 0){
        x = 1;
    }
    unsigned int delayVal = 0xFFFFFFU;
    while(x*delayVal)
    {
        delayVal--;
    }
}
#endif