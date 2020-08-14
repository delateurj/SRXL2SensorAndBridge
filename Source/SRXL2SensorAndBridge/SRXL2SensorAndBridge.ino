/*
MIT License

Copyright (c) 2019 Horizon Hobby, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// This example is not intended to compile -- it just shows an example of how to use spm_srxl.c/.h
// This example is intended to go with the example spm_srxl_config.h to provide guidance on usage.

#include "spm_srxl.h"

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 50

#define srxl2port Serial1
// Example external functions -- imagine these are defined in uart.h and implemented in uart.c
void uartInit(uint8_t uartNum, uint32_t baudRate);
void uartSetBaud(uint8_t uartNum, uint32_t baudRate)
{
}

// UART receive buffer
uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
uint8_t rxBufferIndex = 0;
unsigned long prevSerialRxMillis = 0;
unsigned long prevSerialRxMicros = 0;
unsigned long prevUartMicros = 0;
unsigned long timeLastPacketParse = 0;
unsigned long loopStartTime = 0;
unsigned int timeAtRx = 0;
unsigned int rxCount = 0;
unsigned int txCount = 0;
unsigned int rxOrTxCount = 0;

void uartTransmit(uint8_t uart, uint8_t *pBuffer, uint8_t length)
{
    unsigned int timeAtTx = micros();

    for (uint8_t i = 0; i < length; i++)
    {
        srxl2port.write(pBuffer[i]);
    }
    srxl2port.flush();

    char formattedValue[5];
    rxOrTxCount++;
    Serial.print("TX,");
    sprintf(formattedValue, "%4u", rxOrTxCount);
    Serial.print(formattedValue);
    Serial.print(",");
    sprintf(formattedValue, "%9u", timeAtTx);
    Serial.print(formattedValue);
    for (uint8_t i = 0; i < length; i++)
    {
        Serial.print(",");
        sprintf(formattedValue, "%3X", pBuffer[i]);
        Serial.print(formattedValue);
    }
    Serial.println("");
    prevUartMicros = micros();
}

int LEDPin = 13;

bool LEDOn = false;

void toggleLED()
{
    LEDOn = !LEDOn;
    digitalWrite(LEDPin, LEDOn);
}
// Sample SRXL app
void setup()
{

    pinMode(LEDPin, OUTPUT);
    //Initialize usb port for debugging...baud doesn't matter as it negotiates
    //rate on startup

    Serial.begin(9600);

    // Initialize uart port which we are using Serial1 on Teensy
    Serial1.begin(SRXL2_PORT_BAUDRATE_DEFAULT);

    // Init the local SRXL device
    // Unique ID just need to be "likely" unique...using 0x01000001.example app suggests using rand
    srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001);

    // Init the SRXL bus: The bus index must always be < SRXL_NUM_OF_BUSES -- in this case, it can only be 0
    //Second parameter, uart can be set to anything since our UART transmit hard codes the uart to use
    //to be srxl2port which is Serial1
    srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES); // Init the SRXL bus: The bus index must always be < SRXL_NUM_OF_BUSES -- in this case, it can only be 0 since we have only 1 bus.
}

void loop()
{
    loopStartTime = millis();
    if (loopStartTime - prevSerialRxMillis > SRXL2_FRAME_TIMEOUT)
    {
        prevSerialRxMillis = loopStartTime;
        rxBufferIndex = 0;
        srxlRun(0, SRXL2_FRAME_TIMEOUT);
        Serial.print(loopStartTime);
        Serial.println(": Timeout");
    }

    if (loopStartTime - timeLastPacketParse > 1000)
    {
        digitalWrite(LEDPin, false);
    }
    else
    {
        digitalWrite(LEDPin, true);
    }

    if (srxl2port.available())
    {

        prevSerialRxMillis = loopStartTime;

        unsigned char c = srxl2port.read();

        rxBuffer[rxBufferIndex++] = c;
    }

    if (rxBufferIndex >= 5)
    {

        if (rxBuffer[0] == SPEKTRUM_SRXL_ID)
        {
            uint8_t packetLength = rxBuffer[2];
            if (rxBufferIndex >= packetLength)
            {
                timeLastPacketParse = loopStartTime;

                // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
                if (srxlParsePacket(0, rxBuffer, packetLength))
                {
                    char formattedValue[5];
                    timeAtRx = micros();
                    rxOrTxCount++;
                    Serial.print("RX,");
                    sprintf(formattedValue, "%4u", rxOrTxCount);
                    Serial.print(formattedValue);
                    Serial.print(",");
                    sprintf(formattedValue, "%9u", timeAtRx);
                    Serial.print(formattedValue);
                    for (uint8_t i = 0; i < packetLength; i++)
                    {
                        Serial.print(",");
                        sprintf(formattedValue, "%3X", rxBuffer[i]);
                        Serial.print(formattedValue);
                    }
                    Serial.println("");
                    // Move any remaining bytes to beginning of buffer (usually 0)
                    rxBufferIndex -= packetLength;
                    memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
                }
                else
                {
                    rxBufferIndex = 0;
                }
            }
        }
    }
}

void serialDebug(char message[20])
{
    Serial.println(message);
}

// User-defined routine to populate telemetry data
void userProvidedFillSrxlTelemetry(SrxlTelemetryData *pTelemetry)
{
    // Copy in whatever telemetry data you wish to send back this cycle
    // You can fill it via pTelemetry...
    /*    memset(pTelemetry->raw, 0, 16);
    // ... or directly access the global value srxlTelemData
    */
    static unsigned long fillCount = 0;
    fillCount++;
    pTelemetry->sensorID = 0x7E;

    pTelemetry->data[0] = highByte(6000);
    pTelemetry->data[1] = lowByte(6000);
    pTelemetry->data[2] = highByte(6000);
    pTelemetry->data[3] = lowByte(6000);
    pTelemetry->data[4] = highByte(fillCount);
    pTelemetry->data[5] = lowByte(fillCount);
}

// User-defined routine to use the provided channel data from the SRXL bus master
void userProvidedReceivedChannelData(SrxlChannelData *pChannelData, bool isFailsafeData)
{
    // Use the received channel data in whatever way desired.
    // The failsafe flag is set if the data is failsafe data instead of normal channel data.
    // You can directly access the last received packet data through pChannelData,
    // in which case the values are still packed into the beginning of the array of channel data,
    // and must be identified by the mask bits that are set (NOT recommended!).
    // This is mostly provided in case you want to know which channels were actually sent
    // during this packet:
    /* uint8_t currentIndex = 0;
    if (pChannelData->mask & 1)
        servoA_12bit = pChannelData->values[currentIndex++] >> 4;
    if (pChannelData->mask & 2)
        servoB_12bit = pChannelData->values[currentIndex++] >> 4;

    // The recommended method is to access all channel data through the global srxlChData var:
    servoA_12bit = srxlChData.values[0] >> 4;
    servoB_12bit = srxlChData.values[1] >> 4; */

    //Output throttle command converted from 16 to 10 bit;
    //Serial.println(srxlChData.values[0] >> 6);
}
