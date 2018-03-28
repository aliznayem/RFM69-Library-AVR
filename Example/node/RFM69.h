// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************


// **********************************************************************************
// converted to avr environment by Zulkar Nayem

// microcontroller : atmega64
// default pins:
// MOSI -> PB2
// MISO -> PB3
// SS -> PB0
// DIO0 -> PE5 that is INT5, an interrupt enabled pin
// **********************************************************************************


#ifndef RFM69_h
#define RFM69_h

// Definitions
#define SS_DDR                DDRB
#define SS_PORT              PORTB
#define SS_PIN                 PB0

#define INT_DDR               DDRE
#define INT_PORT             PORTE
#define INT_PIN                PE5
#define INTn                  INT5
#define ISCn0                ISC50
#define ISCn1                ISC51
#define INT_VECT         INT5_vect

#define RF69_MAX_DATA_LEN       61  // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT             -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0   // XTAL OFF
#define RF69_MODE_STANDBY       1   // XTAL ON
#define RF69_MODE_SYNTH         2   // PLL ON
#define RF69_MODE_RX            3   // RX MODE
#define RF69_MODE_TX            4   // TX MODE
#define null                  0
#define COURSE_TEMP_COEF    -90     // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP    61.035156     // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet) FXOSC = module crystal oscillator frequency 
// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40
    
// Function Declerations
void rfm69_init(uint16_t freqBand, uint8_t nodeID, uint8_t networkID=33);
void setAddress(uint8_t addr);
void setNetwork(uint8_t networkID);
uint8_t canSend();
void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t requestACK=0);
uint8_t sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);
uint8_t ACKRequested();
uint8_t ACKReceived(uint8_t fromNodeID);
void receiveBegin();
uint8_t receiveDone();
void sendACK(const void* buffer = "", uint8_t bufferSize=0);
uint32_t getFrequency();
void setFrequency(uint32_t freqHz);
void encrypt(const char* key);
int16_t readRSSI(uint8_t forceTrigger=0);
void setHighPower(uint8_t onOFF=1);           // has to be called after initialize() for RFM69HW
void setPowerLevel(uint8_t level);            // reduce/increase transmit power level
void sleep();
uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
void rcCalibration();                         // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]
uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t val);
void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, uint8_t requestACK=0, uint8_t sendACK=0);
void setMode(uint8_t mode);
void setHighPowerRegs(uint8_t onOff);
void promiscuous(uint8_t onOff);
void maybeInterrupts();
void select();
void unselect();
uint8_t receiveDone();

#endif

