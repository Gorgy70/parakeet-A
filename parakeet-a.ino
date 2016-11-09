#include <SPI.h>

#include "cc2500_REG.h"

#define SCAN_CS      10     // scanner Select
#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов

byte fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
unsigned long waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
unsigned long delayedWaitTimes[NUM_CHANNELS] = { 0, 700, 700, 700 };
unsigned long catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };

typedef struct _Dexcom_packet
{
    byte len;
    unsigned long dest_addr;
    unsigned long src_addr;
    byte port;
    byte device_info;
    byte txId;
    unsigned int raw;
    unsigned int filtered;
    byte battery;
    byte unknown;
    byte checksum;
    byte RSSI;
    byte LQI2;
} Dexcom_packet;

Dexcom_packet Pkt;

void WriteReg(char addr, char value){
  digitalWrite(SCAN_CS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SCAN_CS,HIGH);
}

void init_CC2500(){
  WriteReg(SRES,0x3D);       // software reset for CC2500
  WriteReg(FSCTRL1,0x0F);    // Frequency Synthesizer Control
  WriteReg(PKTCTRL0,0x12);   // Packet Automation Control
  WriteReg(FREQ2,0x5C);      // Frequency control word, high byte
  WriteReg(FREQ1,0x4E);      // Frequency control word, middle byte
  WriteReg(FREQ0,0xDE);      // Frequency control word, low byte
  WriteReg(MDMCFG4,0x0D);    // Modem Configuration
  WriteReg(MDMCFG3,0x3B);    // Modem Configuration
  WriteReg(MDMCFG2,0x00);    // Modem Configuration 0x30 - OOK modulation, 0x00 - FSK modulation (better sensitivity)
  WriteReg(MDMCFG1,0x23);    // Modem Configuration
  WriteReg(MDMCFG0,0xFF);    // Modem Configuration
  WriteReg(MCSM1,0x0F);      // Always stay in RX mode
  WriteReg(MCSM0,0x04);      // Main Radio Control State Machine Configuration
  WriteReg(FOCCFG,0x15);     // Frequency Offset Compensation configuration
  WriteReg(AGCCTRL2,0x83);   // AGC Control
  WriteReg(AGCCTRL1,0x00);   // AGC Control
  WriteReg(AGCCTRL0,0x91);   // AGC Control
  WriteReg(FSCAL3,0xEA);     // Frequency Synthesizer Calibration
  WriteReg(FSCAL2,0x0A);     // Frequency Synthesizer Calibration 
  WriteReg(FSCAL1,0x00);     // Frequency Synthesizer Calibration 
  WriteReg(FSCAL0,0x11);     // Frequency Synthesizer Calibration
}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SCAN_CS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SCAN_CS,HIGH);
  return y;  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK
  digitalWrite(SCAN_CS,HIGH);
  Serial.println("CC250 init start ....");
  init_CC2500();  // initialise CC2500 registers
  Serial.println("CC250 init OK");
  memset (&Pkt, 0, sizeof (Dexcom_packet));
}

boolean WaitForPacket(unsigned int milliseconds, Dexcom_packet * pkt, byte channel)
{
  
}
boolean get_packet (Dexcom_packet * pPkt) {
  int nChannel = 0;
  for (nChannel = 0; nChannel < NUM_CHANNELS; nChannel++)
  {
    if (WaitForPacket (delayFor(nChannel), pPkt, nChannel)) {
      return true;
    }
  }
  return false;
}

void print_packet(Dexcom_packet * pPkt) {
  
}

void loop() {
  if (get_packet (&Pkt))
  {
     print_packet (&Pkt);
  }

}



