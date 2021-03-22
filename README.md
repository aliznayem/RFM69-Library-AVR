# RFM69-Library-AVR #
Original library is written for arduino by [LowPowerLab](https://github.com/LowPowerLab/RFM69). This is C ported version for AVR microcontrollers.

Tested on ATmega64, ATmega328, ATtiny84(as receiver).

</br>

## I/O pin connections: ##

| RF Module | Microcontroller |
| --------- | --------------- |
|MOSI |	MOSI |
| MISO | MISO |
| SCK | SCK |
| SS | SS |
| DIO0 | Any interrupt enabled pin |

</br>

## Function Description: ##

1.	**rfm69_init(*uint16_t freqBand, uint8_t nodeID, uint8_t networkID*):** Initializes rfm69 module. This function is called at the beginning of the program. Initializes IDs, modes etc. It takes three parameters. First one freqBand. You have to choose among 315, 433, 868 and 915. These specifies frequency in MHz. nodeID is analogues to device ID. Each RF module will have unique nodeID. Value must be within 0 to 255. Then comes notworkID. Say, a system has 5 rf modules to communicate with each other. All the modules must be in same networkID . networkID value range 0~255.
2.	**setAddress(*uint8_t addr*):** Sets nodeID.
3.	**setNetwork(*uint8_t networkID*):** Sets networkID.
4.	**send(*uint8_t toAddress, const void\* buffer, uint8_t bufferSize, uint8_t requestACK*):** Transmits data to another node. First argument is toAddress that is address off receiver node/gateway. In buffer you can put any kind of buffer like string or array etc. In requestACK you can pass 0 or 1 whether you need acknowledgement of transmitted data.
5.	**sendWithRetry(*uint8_t toAddress, const void\* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime*):** This sends data with retry. Internally manages ACK. retryWaitTime – after transmitting data module waits for ack if doesn’t have ack then again transmits data. retryWaitTime is time interval between sending.
6.	**ACKRequested():** This function needed in listening process. Checks whether acknowledgement requested or not.
7.	**sendACK(*const void\* buffer , uint8_t bufferSize*):** If ACK requested, send ACK through this function.
8.	**receiveDone():**  Returns 1 if any data is present in receive buffer.
9.	**getFrequency():** Gets frequency Band.
10.	**setFrequency(*uint32_t freqHz*):** Sets frequency band. You can set frequency other than 315, 433, 868, 915 MHz through this function. Unit is Hz i.e 433000000. 
11.	**encrypt(*const char\* key*):** All device need same encryption key. And length must be 16. If you need no encryption just put 0 in argument. 
12.	**readRSSI(*uint8_t forceTrigger=0*):** You want to know received signal strength? :D
13.	**setHighPower(*uint8_t onOFF=1*):** RFM69 has different suffixes like, W, HW or HCW etcetra. Having ‘H’ word indicated high power enabled. If you use module having ‘H’ letter put 1 as argument. This function must be called after initialization.
14.	**setPowerLevel(*uint8_t level*):** Sets transmit power. Range 0~31.
15.	**readTemperature(*uint8_t calFactor=0*):** gets CMOS temperature (8bit)
16.	**rcCalibration():** Calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section 4.3.5. RC Timer Accuracy. Not tested yet.
17.	**promiscuous(*uint8_t onOff*):** 1 or 0. If on, module receives data indiscriminately. In another word, it receives all data in network when set to 1.

</br>

## Basic Operation: ##
#### Transmit data: ####

```
rfm69_init(freq, nodeID, networkID)
setHighPower(0 or 1)
setPowerLevel(0~31)
send(toNodeID, buffer, bufferLen, 0 or 1 )
```

#### Receive data: ####

```
rfm69_init(freq, nodeID, networkID)
setHighPower(0 or 1)
setPowerLevel(0~31)
mainloop
    if receiveDone() then
        if ACKRequested() then
            sendACK()
        extract received data from DATA buffer
```
