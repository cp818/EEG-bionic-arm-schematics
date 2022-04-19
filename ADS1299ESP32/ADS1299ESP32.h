//
//  ADS1299.h
//  Part of the Arduino Library
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer 2013.
//
//  Modified by Chip Audette through April 2014
//

#ifndef ____ADS1299ESP32__
#define ____ADS1299ESP32__



//#include <C:\Users\Momin\Documents\ArduinoData\packages\esp32\hardware\esp32\1.0.4\cores\esp32\pgmspace.h>
#include <stdio.h>
#include <Arduino.h>
#include <SPI.h>
#include "SPI.h"
#include "DefinitionsESP32.h"
  

class ADS1299ESP32 {
public:
    
    void initialize();
    
    //ADS1299 SPI Command Definitions (Datasheet, p35)
    //System Commands
    void wakeup();
    void standby();
    void reset();
    void start();
    void stop();
    
    //Data Read Commands
    void rdatac();
    void sdatac();
    void rdata();
    
    //Register Read/Write Commands
    byte getID();
    byte rreg(byte _address);
    void rregs(byte _address, byte _numRegistersMinusOne);     
    void printRegisterName(byte _address);
    void wreg(byte _address, byte _value); 
    void wregs(byte _address, byte _numRegistersMinusOne); 
    void printHex(byte _data);
    void updatedata();
    

    //configuration
    int DRDY, CS; 		// pin numbers for DRDY and CS 
    int DIVIDER;		// select SPI SCK frequency
    int stat_1, stat_2;    // used to hold the status register for boards 1 and 2
    byte regData [24];	// array is used to mirror register data
    long channelData [8];	// array used when reading channel data board 1+2
    boolean disp;		// turn on/off Serial feedback
    boolean isDaisy;		// does this have a daisy chain board?
    
    
};

#endif