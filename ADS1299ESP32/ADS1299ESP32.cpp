//
//  ADS1299DIAISY.cpp   ARDUINO LIBRARY FOR COMMUNICATING WITH TWO
//  DAISY-CHAINED ADS1299 BOARDS
//  
//  Created by Conor Russomanno, Luke Travis, and Joel Murphy. Summer, 2013
//
//  Extended by Chip Audette through April 2014
//


//#include "pins_arduino.h"
#include "ADS1299ESP32.h"

static const uint8_t SCLK = 14;
static const uint8_t CS   = 15;
static const uint8_t DRDY = 33;
static const uint8_t RST = 32;

static const int spiClk = 4000000;

SPIClass *hspi = NULL;
//hspi = new SPIClass(HSPI);



void ADS1299ESP32::initialize(){

  hspi->begin(); 
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  pinMode(CS,OUTPUT);
  pinMode(DRDY, INPUT); 
  digitalWrite(CS,HIGH);
  delay(50);
  pinMode(RST, OUTPUT);
  digitalWrite(RST,LOW);
  delayMicroseconds(4);
  digitalWrite(RST,HIGH);
  delayMicroseconds(20);  
  hspi->endTransaction();

}

//System Commands
void ADS1299ESP32::wakeup() {
    digitalWrite(CS, LOW); 
    hspi->transfer(_WAKEUP);
    delayMicroseconds(12);
    digitalWrite(CS, HIGH); 
    delayMicroseconds(3);  		//must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
}

void ADS1299ESP32::standby() {		// only allowed to send WAKEUP after sending STANDBY
    digitalWrite(CS, LOW);
    hspi->transfer(_STANDBY);
    delayMicroseconds(12);
    digitalWrite(CS, HIGH);
}

void ADS1299ESP32::reset()
{
  digitalWrite(CS,LOW);
  hspi->transfer(_RESET);
  delayMicroseconds(12);
  digitalWrite(CS,HIGH);
}

void ADS1299ESP32::start()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(3);
  hspi->transfer(_START);
  digitalWrite(CS,HIGH);
  delayMicroseconds(3);
}

void ADS1299ESP32::stop() {			//stop data conversion
    digitalWrite(CS, LOW);
    delayMicroseconds(3);
    hspi->transfer(_START);
    digitalWrite(CS, HIGH);
    delayMicroseconds(3);
}

void ADS1299ESP32::rdatac()
{
  digitalWrite(CS,LOW);
  delayMicroseconds(3);
  hspi->transfer(_RDATAC);
  digitalWrite(CS, HIGH);
  delayMicroseconds(3);
}

void ADS1299ESP32::sdatac() 
{
  digitalWrite(CS,LOW);
  delayMicroseconds(2);
  hspi->transfer(_SDATAC);
  digitalWrite(CS,HIGH);
  delayMicroseconds(3);
}


// Register Read/Write Commands
byte ADS1299ESP32::getID()
{
  byte data = rreg(0x00);
  if(disp){
    Serial.print(F("Device ID "));
    printHex(data); 
  }
  return data;
}

byte ADS1299ESP32::rreg(byte address)// Read one register starting at _address
{
  byte opcode1 = address + 0x20;
  digitalWrite(CS,LOW);
  hspi->transfer(opcode1);
  delayMicroseconds(2);
  hspi->transfer(0x00);
  regData[address] = hspi->transfer(0x00);
  digitalWrite(CS,HIGH);

  if(disp)
  {
    printRegisterName(address);
    printHex(address);
    Serial.print(", ");
    printHex(regData[address]);
    Serial.print(", ");
    for(byte j = 0; j<8; j++){
      Serial.print(bitRead(regData[address], 7-j));
      if(j!=7) Serial.print(", ");
    }   
    Serial.println();
  }
  return regData[address];
}

// Read more than one register starting at _address

void ADS1299ESP32::rregs(byte address, byte numRegistersMinusOne) 
{
    byte opcode1 = address + 0x20;
    digitalWrite(CS, LOW);
    hspi->transfer(opcode1); 
    hspi->transfer(numRegistersMinusOne); 
    for(int i = 0; i <= numRegistersMinusOne; i++)
    {
      regData[address + i] = hspi->transfer(0x00); 
    }
    digitalWrite(CS, HIGH);
    if(disp)
    { 
      for(int i = 0; i<= numRegistersMinusOne; i++)
      {
        printRegisterName(address + i);
        printHex(address + i);
        Serial.print(", ");
        printHex(regData[address + i]);
        Serial.print(", ");
        for(int j = 0; j<8; j++)
        {
          Serial.print(bitRead(regData[address + i], 7-j));
          if(j!=7) Serial.print(", ");
        }
        Serial.println();
      }
     }   
}
void ADS1299ESP32::wreg(byte _address, byte _value) 
{  
    byte opcode1 = _address + 0x40;   
    digitalWrite(CS, LOW);   
    hspi->transfer(opcode1);        
    hspi->transfer(0x00);       
    hspi->transfer(_value);     
    digitalWrite(CS, HIGH);     
  regData[_address] = _value;   
  if(disp){  
    Serial.print(F("Register "));
    printHex(_address);
    Serial.println(F(" modified."));
  }
}
void ADS1299ESP32::wregs(byte address, byte numRegistersMinusOne) 
{
    byte opcode1 = address + 0x40;   
    digitalWrite(CS, LOW);       
    hspi->transfer(opcode1);   
    hspi->transfer(numRegistersMinusOne); 
    for (int i=address; i <=(address + numRegistersMinusOne); i++)
    {
      hspi->transfer(regData[i]);    
    } 
    digitalWrite(CS,HIGH);  
    if(disp)
    {
      Serial.print(F("Registers "));
      printHex(address); Serial.print(F(" to "));
      printHex(address + numRegistersMinusOne);
      Serial.println(F(" modified"));
    }
}
void ADS1299ESP32::updatedata(){
 
  byte inByte;
  int nchan=8;
  digitalWrite(CS, LOW);
  delayMicroseconds(2);
  for(int i=0; i<3; i++)
  { 
    inByte = hspi->transfer(0x00);
    stat_1 = (stat_1<<8) | inByte;        
  }
  
  for(int i = 0; i<8; i++)
  {
    for(int j=0; j<3; j++)
    {
      inByte = hspi->transfer(0x00);
      channelData[i] = (channelData[i]<<8) | inByte;
    }
  }
  digitalWrite(CS, HIGH); 
  
  for(int i=0; i<nchan; i++)
  {     
    if(bitRead(channelData[i],23) == 1)
    {  
      channelData[i] |= 0xFF000000;
    }
    else
    {
      channelData[i] &= 0x00FFFFFF;
    }
  }
}



void ADS1299ESP32::printRegisterName(byte _address) 
{
    if(_address == ID){
        Serial.print(F("ID, "));
    }
    else if(_address == CONFIG1){
        Serial.print(F("CONFIG1, "));
    }
    else if(_address == CONFIG2){
        Serial.print(F("CONFIG2, "));
    }
    else if(_address == CONFIG3){
        Serial.print(F("CONFIG3, "));
    }
    else if(_address == LOFF){
        Serial.print(F("LOFF, "));
    }
    else if(_address == CH1SET){
        Serial.print(F("CH1SET, "));
    }
    else if(_address == CH2SET){
        Serial.print(F("CH2SET, "));
    }
    else if(_address == CH3SET){
        Serial.print(F("CH3SET, "));
    }
    else if(_address == CH4SET){
        Serial.print(F("CH4SET, "));
    }
    else if(_address == CH5SET){
        Serial.print(F("CH5SET, "));
    }
    else if(_address == CH6SET){
        Serial.print(F("CH6SET, "));
    }
    else if(_address == CH7SET){
        Serial.print(F("CH7SET, "));
    }
    else if(_address == CH8SET){
        Serial.print(F("CH8SET, "));
    }
    else if(_address == BIAS_SENSP){
        Serial.print(F("BIAS_SENSP, "));
    }
    else if(_address == BIAS_SENSN){
        Serial.print(F("BIAS_SENSN, "));
    }
    else if(_address == LOFF_SENSP){
        Serial.print(F("LOFF_SENSP, "));
    }
    else if(_address == LOFF_SENSN){
        Serial.print(F("LOFF_SENSN, "));
    }
    else if(_address == LOFF_FLIP){
        Serial.print(F("LOFF_FLIP, "));
    }
    else if(_address == LOFF_STATP){
        Serial.print(F("LOFF_STATP, "));
    }
    else if(_address == LOFF_STATN){
        Serial.print(F("LOFF_STATN, "));
    }
    else if(_address == GPIO){
        Serial.print(F("GPIO, "));
    }
    else if(_address == MISC1){
        Serial.print(F("MISC1, "));
    }
    else if(_address == MISC2){
        Serial.print(F("MISC2, "));
    }
    else if(_address == CONFIG4){
        Serial.print(F("CONFIG4, "));
    }
}

 

// Used for printing HEX in verbose feedback mode
void ADS1299ESP32::printHex(byte _data){
  Serial.print("0x");
    if(_data < 0x10) Serial.print("0");
    Serial.print(_data, HEX);
}

//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//



