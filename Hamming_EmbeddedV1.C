/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */
#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xuartps.h"

//------------------------------------------------------------------------------
// Helper function: Print a value in binary with the specified width.
//------------------------------------------------------------------------------
static void printBinary(u32 value, int width)
{
    for (int i = width - 1; i >= 0; i--)
    {
        xil_printf("%c", (value & (1 << i)) ? '1' : '0');
    }
}

//------------------------------------------------------------------------------
// Helper function: Inject an error by flipping a specified bit (0-indexed)
// in the 7-bit encoded data.
//------------------------------------------------------------------------------
static u8 injectError(u8 encodedData, int bitToFlip)
{
    // Ensure bitToFlip is within [0,6]
    if (bitToFlip >= 0 && bitToFlip < 7)
    {
        encodedData ^= (1 << bitToFlip);
    }
    return encodedData;
}

//------------------------------------------------------------------------------
// Main function
//------------------------------------------------------------------------------
int main(void)
{
    //==========================================================================
    // Variable Declarations
    //==========================================================================
    // AXI GPIO 0 (Encoder) Variables
    XGpio gpioEncoder;
    int statusEncoder;
    u8 inputChar;
    u8 nibbleValue;
    u8 encoderOutValue;   // 6-bit word for GPIO0 Channel 1
    u32 encodedRaw;       // 9-bit encoded output from GPIO0 Channel 2
    u8 ack, ready;
    u8 encodedData;       // 7-bit encoded data

    // AXI GPIO 1 (Decoder) Variables
    XGpio gpioDecoder;
    int statusDecoder;
    u32 decoderRaw;       // 8-bit read from GPIO1 Channel 2
    // Decoder output mapping (from de_out_slice):
    // Bit 7: ready, Bit 6: error_detected, Bit 5: error_corrected,
    // Bit 4: uncorrectable_error, Bits [3:0]: decoded 4-bit data.
    u8 decoderReady, errDetected, errCorrected, uncorrectable;
    u8 decodedData;
    u32 decoderInValue;   // 9-bit word to send to decoder (via GPIO1 Channel 1)

    // Variables for Error Injection Menu
    u8 choice;
    int numErrors;
    int bitIndex;
    char injectionChoice; // 'P' for parity, 'D' for data

    //==========================================================================
    // Initialize AXI GPIO 0 (Encoder)
    //==========================================================================
    statusEncoder = XGpio_Initialize(&gpioEncoder, XPAR_AXI_GPIO_0_DEVICE_ID);
    if (statusEncoder != XST_SUCCESS)
    {
        xil_printf("AXI GPIO 0 (encoder) initialization failed!\r\n");
        return statusEncoder;
    }
    // Configure GPIO 0:
    // - Channel 1 as output (6-bit word)
    // - Channel 2 as input (9-bit word)
    XGpio_SetDataDirection(&gpioEncoder, 1, 0x00);
    XGpio_SetDataDirection(&gpioEncoder, 2, 0xFF);

    //==========================================================================
    // Initialize AXI GPIO 1 (Decoder)
    //==========================================================================
    statusDecoder = XGpio_Initialize(&gpioDecoder, XPAR_AXI_GPIO_1_DEVICE_ID);
    if (statusDecoder != XST_SUCCESS)
    {
        xil_printf("AXI GPIO 1 (decoder) initialization failed!\r\n");
        return statusDecoder;
    }
    // Configure GPIO 1:
    // - Channel 1 as output (9-bit word for de_in_slice)
    // - Channel 2 as input (8-bit word from de_out_slice)
    XGpio_SetDataDirection(&gpioDecoder, 1, 0x00);
    XGpio_SetDataDirection(&gpioDecoder, 2, 0xFF);

    //==========================================================================
    // Main Loop: Process User Input, Inject Errors, and Route to Decoder
    //==========================================================================
    while (1)
    {
        //************************* Encoder Input *************************
        xil_printf("\r\nEnter a 4-bit hex value (0-9, A-F):\r\n");
        while (!XUartPs_IsReceiveData(UART_BASEADDR))
            ;  // Busy-wait for UART input

        // Read and echo user input
        inputChar = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, inputChar);

        // Convert ASCII character to a 4-bit nibble
        if (inputChar >= '0' && inputChar <= '9')
            nibbleValue = inputChar - '0';
        else if (inputChar >= 'A' && inputChar <= 'F')
            nibbleValue = inputChar - 'A' + 10;
        else if (inputChar >= 'a' && inputChar <= 'f')
            nibbleValue = inputChar - 'a' + 10;
        else
        {
            xil_printf("\r\nInvalid hex digit!\r\n");
            continue;
        }

        //----- Encoder Processing (AXI GPIO 0) -----
        // Reset the encoder (valid=0, nrst=0)
        encoderOutValue = 0x00;
        XGpio_DiscreteWrite(&gpioEncoder, 1, (u32)encoderOutValue);
        for (int j = 0; j < 1000; j++) { asm volatile("nop"); }

        // Send new input: set bits [3:0]=nibbleValue, valid (bit 4)=1, nrst (bit 5)=1
        encoderOutValue = (nibbleValue & 0x0F) | (1 << 4) | (1 << 5);
        XGpio_DiscreteWrite(&gpioEncoder, 1, (u32)encoderOutValue);
        asm volatile("nop"); asm volatile("nop");

        // Read encoded output from encoder (AXI GPIO 0, Channel 2)
        encodedRaw = XGpio_DiscreteRead(&gpioEncoder, 2);
        ack         = (encodedRaw >> 8) & 0x1;
        ready       = (encodedRaw >> 7) & 0x1;
        encodedData = encodedRaw & 0x7F;

        xil_printf("\r\n[Encoder Results]\r\n");
        xil_printf("User Input         = b'");
        printBinary(nibbleValue, 4);
        xil_printf("'\r\n");
        xil_printf("Encoded Data Out   = b'");
        printBinary(encodedData, 7);
        xil_printf("'\r\n");
        xil_printf("Acknowledge        = '%c'\r\n", (ack ? '1' : '0'));
        xil_printf("Ready              = '%c'\r\n", (ready ? '1' : '0'));

        //**************** Error Injection Menu ****************
        xil_printf("\r\nDo you want to inject error bits? (Y/N):\r\n");
        while (!XUartPs_IsReceiveData(UART_BASEADDR))
            ;
        choice = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, choice);

        // If user chooses to inject errors...
        if (choice == 'Y' || choice == 'y')
        {
            xil_printf("\r\n****************************************\r\n");
            xil_printf("*       ERROR INJECTION MENU           *\r\n");
            xil_printf("*  Enter number of error bits (1-3):     *\r\n");
            xil_printf("****************************************\r\n");
            while (!XUartPs_IsReceiveData(UART_BASEADDR))
                ;
            inputChar = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
            XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, inputChar);
            numErrors = inputChar - '0';
            if (numErrors < 1 || numErrors > 3)
            {
                xil_printf("\r\nInvalid number. Defaulting to 1 error.\r\n");
                numErrors = 1;
            }

            xil_printf("\r\nDo you want to inject errors into (P)arity bits or (D)ata bits? (P/D):\r\n");
            while (!XUartPs_IsReceiveData(UART_BASEADDR))
                ;
            injectionChoice = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
            XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, injectionChoice);
            // Convert to uppercase if necessary.
            if (injectionChoice >= 'a' && injectionChoice <= 'z')
                injectionChoice -= 32;

            // For each error to be injected:
            for (int i = 0; i < numErrors; i++)
            {
                if (injectionChoice == 'P')
                {
                    xil_printf("\r\nEnter error bit index to flip (choose from 3, 1, or 0):\r\n");
                    while (!XUartPs_IsReceiveData(UART_BASEADDR))
                        ;
                    inputChar = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
                    XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, inputChar);
                    bitIndex = inputChar - '0';
                    if (!(bitIndex == 3 || bitIndex == 1 || bitIndex == 0))
                    {
                        xil_printf("\r\nInvalid bit index. Defaulting to 3.\r\n");
                        bitIndex = 3;
                    }
                }
                else if (injectionChoice == 'D')
                {
                    xil_printf("\r\nEnter error bit index to flip (choose from 6, 5, 4, or 2):\r\n");
                    while (!XUartPs_IsReceiveData(UART_BASEADDR))
                        ;
                    inputChar = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
                    XUartPs_WriteReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET, inputChar);
                    bitIndex = inputChar - '0';
                    if (!(bitIndex == 6 || bitIndex == 5 || bitIndex == 4 || bitIndex == 2))
                    {
                        xil_printf("\r\nInvalid bit index. Defaulting to 6.\r\n");
                        bitIndex = 6;
                    }
                }
                else
                {
                    xil_printf("\r\nInvalid choice. Defaulting to Parity bits.\r\n");
                    injectionChoice = 'P';
                    bitIndex = 3;
                }
                // Inject error into the encoded data.
                encodedData = injectError(encodedData, bitIndex);
            }
            xil_printf("\r\n[After Error Injection] Encoded Data Out = b'");
            printBinary(encodedData, 7);
            xil_printf("'\r\n");
        }
        else
        {
            xil_printf("\r\nNo errors injected.\r\n");
        }

        //------------------ Routing to the Decoder (AXI GPIO 1) ------------------
        // Build a 9-bit value for the decoder:
        // Set: nrst = 1 (bit 8), valid = 1 (bit 7), data = encodedData (bits [6:0]).
        decoderInValue = (1 << 8) | (1 << 7) | (encodedData & 0x7F);
        XGpio_DiscreteWrite(&gpioDecoder, 1, decoderInValue);
        // Hold valid high briefly, then deassert valid.
        for (int j = 0; j < 5000; j++) { asm volatile("nop"); }
        decoderInValue = (1 << 8) | (0 << 7) | (encodedData & 0x7F);
        XGpio_DiscreteWrite(&gpioDecoder, 1, decoderInValue);
        for (int j = 0; j < 5000; j++) { asm volatile("nop"); }

        //------------------ Read Decoder Output (AXI GPIO 1) ------------------
        decoderRaw = XGpio_DiscreteRead(&gpioDecoder, 2) & 0xFF;
        // Mapping from de_out_slice:
        // Bit 7: ready, Bit 6: error_detected, Bit 5: error_corrected,
        // Bit 4: uncorrectable_error, Bits [3:0]: decoded 4-bit data.
        decoderReady   = (decoderRaw >> 7) & 0x1;
        errDetected    = (decoderRaw >> 6) & 0x1;
        errCorrected   = (decoderRaw >> 5) & 0x1;
        uncorrectable  = (decoderRaw >> 4) & 0x1;
        decodedData    = decoderRaw & 0x0F;

        xil_printf("\r\n[Decoder Results]\r\n");
        xil_printf("Decoded Data       = b'");
        printBinary(decodedData, 4);
        xil_printf("'\r\n");
        xil_printf("Ready              = '%c'\r\n", (decoderReady ? '1' : '0'));
        xil_printf("Error Detected     = '%c'\r\n", (errDetected ? '1' : '0'));
        xil_printf("Error Corrected    = '%c'\r\n", (errCorrected ? '0' : '1'));
        xil_printf("Uncorrectable Err  = '%c'\r\n", (uncorrectable ? '0' : '1'));

        //----- Optional: Deassert the encoder's valid signal for next transaction -----
        encoderOutValue = (nibbleValue & 0x0F) | (0 << 4) | (1 << 5);
        XGpio_DiscreteWrite(&gpioEncoder, 1, (u32)encoderOutValue);
        for (int j = 0; j < 1000; j++) { asm volatile("nop"); }
    }

    return 0;
}
