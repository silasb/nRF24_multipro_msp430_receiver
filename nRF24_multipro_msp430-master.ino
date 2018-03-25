/*
 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

//#include <util/atomic.h>
//#include <EEPROM.h>
/*+*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//#include <MspFlash.h>
//#define flash SEGMENT_D
#define _BV(val) 1<<val
/*+*/
#include "iface_nrf24l01.h"


// ############ Wiring ################
//SPI Comm.pins with nRF24L01
#define MOSI_pin  P2_0  // MOSI
#define SCK_pin   P2_1  // SCK
#define CE_pin    P1_6  // CE
#define MISO_pin  P2_2 // MISO
#define CS_pin    P2_4 // CS
#define ledPin    P1_0 // LED

// SPI outputs
#define MOSI_on P2OUT |= _BV(0)// P2_0
#define MOSI_off P2OUT &= ~_BV(0)// P2_0
#define SCK_on P2OUT |= _BV(1)// P2_1
#define SCK_off P2OUT &= ~_BV(1)// P2_1
#define CE_on P1OUT |= _BV(6)// P2_3
#define CE_off P1OUT &= ~_BV(6)// P2_3
#define CS_on P2OUT |= _BV(4)// P2_4
#define CS_off P2OUT &= ~_BV(4) // P2_4
// SPI input
#define  MISO_on (P2IN & _BV(2)) // P2_2

#define RF_POWER TX_POWER_80mW

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint8_t transmitterID[4];
uint8_t current_protocol = PROTO_BAYANG;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

void setup()
{
    randomSeed((analogRead(A0) & 0x1F) | (analogRead(A1) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    // sumd speed
    Serial.begin(115200);

     // PPM ISR setup
//    attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
// /*+*/ attachInterrupt(PPM_pin, ISR_ppm, CHANGE);
//    TCCR1A = 0;  //reset timer1
//    TCCR1B = 0;
//    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz
///*+*/ TACTL = TASSEL_2 + ID_3 + MC_2 + TACLR; //16000000 / 8
    set_txid(false);
}

void loop()
{
    uint32_t timeout;
    // reset / rebind
    if(reset) {
        reset = false;
//        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    // process protocol
    switch(current_protocol) {
        case PROTO_BAYANG:
            timeout = process_Bayang();
            break;
    }
    // updates ppm values out of ISR
//    update_ppm();
    // wait before sending next packet
    while(micros() < timeout)
    {   };
}

void set_txid(bool renew)
{
/*+*/    unsigned char p = 0;
    uint8_t i;
//    for(i=0; i<4; i++)
///*+*/ {
//        transmitterID[i] = EEPROM.read(ee_TXID0+i);
///*+*/      Flash.read(flash+ee_TXID0+i,&p,1);
///*+*/      transmitterID[i] =p;
///*+*/}
///*+*/    Flash.read(flash+ee_PROTOCOL_ID,&p,1);
///*+*/    current_protocol = constrain(p,0,PROTO_END-1);
///*+*/    Flash.erase(flash);
//    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
//            transmitterID[i] = random() & 0xFF;
/*+*/         transmitterID[i] = random(0xFF);
//            EEPROM.write(ee_TXID0+i, transmitterID[i]);
///*+*/            p = transmitterID[i];
///*+*/            Flash.write(flash+ee_TXID0+i, &p,1);
        }
//    }
///*+*/    else{
///*+*/        for(i=0; i<4; i++) {
///*+*/            p = transmitterID[i];
///*+*/            Flash.write(flash+ee_TXID0+i, &p,1);
///*+*/        }
///*+*/      }
}

void selectProtocol()
{
    // wait for multiple complete ppm frames
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok) {} // wait
        update_ppm();
        if(ppm[AUX8] < PPM_MAX_COMMAND) // reset chan released
            count--;
        ppm_ok = false;
    }

    // startup stick commands

    if(ppm[RUDDER] < PPM_MIN_COMMAND)        // Rudder left
        set_txid(true);                      // Renew Transmitter ID

    // protocol selection

    // Elevator up + Aileron right
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;    // EAchine H8(C) mini, BayangToys X6/X7/X9, JJRC JJ850 ...

    // read last used protocol from eeprom
//    else
//        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);
    // update eeprom
//    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
///*+*/ Flash.write(flash+ee_PROTOCOL_ID, &current_protocol,1);
     // wait for safe throttle
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_BAYANG:
            Bayang_init();
            Bayang_bind();
            break;
    }
}

// update ppm values out of ISR
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
//        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
/*+*/         __disable_interrupt();
            ppm[ch] = Servo_data[ch];
//        }
/*+*/         __enable_interrupt();
    }
}