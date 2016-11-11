#define DEBUG

#include <SPI.h>
#include <SoftwareSerial.h>
#include "cc2500_REG.h"

#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов
#define RADIO_MAX_PACKET_SIZE  19

unsigned long dex_tx_id;
char transmitter_id[] = "ABCDE";
boolean only_listen_for_my_transmitter = true; // 1 is recommended    



byte fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
unsigned long waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
unsigned long delayedWaitTimes[NUM_CHANNELS] = { 0, 700, 700, 700 };
unsigned long catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };
byte last_catch_channel = 0;
boolean needsTimingCalibration = true;
byte sequential_missed_packets = 0;

//                                                                                                  //
//                 Advanced Options, dont change unless you know what you are doing                 //
//                                                                                                  //
//                                                                                                  //
byte wake_earlier_for_next_miss = 20;
// if a packet is missed, wake this many seconds earlier to try and get the next one                //
// shorter means better bettery life but more likely to miss multiple packets in a row              //
//                                                                                                  //
byte misses_until_failure = 2;                                                   //
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //

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

char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };


void dexcom_src_to_ascii(unsigned long src, char addr[6]) {
    addr[0] = SrcNameTable[(src >> 20) & 0x1F];
    addr[1] = SrcNameTable[(src >> 15) & 0x1F];
    addr[2] = SrcNameTable[(src >> 10) & 0x1F];
    addr[3] = SrcNameTable[(src >> 5) & 0x1F];
    addr[4] = SrcNameTable[(src >> 0) & 0x1F];
    addr[5] = 0;
}


unsigned long getSrcValue(char srcVal) {
    byte i = 0;
    for(i = 0; i < 32; i++) {
        if (SrcNameTable[i]==srcVal) break;
    }
    return i & 0xFF;
}

unsigned long asciiToDexcomSrc(char addr[6]) {
    unsigned long src = 0;
    src |= (getSrcValue(addr[0]) << 20);
    src |= (getSrcValue(addr[1]) << 15);
    src |= (getSrcValue(addr[2]) << 10);
    src |= (getSrcValue(addr[3]) << 5);
    src |= getSrcValue(addr[4]);
    return src;
}

void blink_builtin_led_quarter() {  // Blink quarter seconds
  if ((millis()/250) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);    
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void blink_builtin_led_half() {  // Blink half seconds
  if ((millis()/500) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);    
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void WriteReg(char addr, char value){
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char SendStrobe(char strobe)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}

void init_CC2500(){


  

//  WriteReg(SRES,0x3D);       // software reset for CC2500
  WriteReg(IOCFG0,0x06) ;      // GD0 Asserts when sync word has been sent / received, and de-asserts at the end of the packet. In RX, the pin will de-assert
                               // when the optional address check fails or the RX FIFO overflows. In TX the pin will de-assert if the TX FIFO underflows.
  WriteReg(CHANNR,0x00);

  WriteReg(FREQ2,0x65);      // Frequency control word, high byte. Value from wixel project
  WriteReg(FREQ1,0x0A);      // Frequency control word, middle byte. Value from wixel project
  WriteReg(FREQ0,0xAA);      // Frequency control word, low byte. Value from wixel project
  
  WriteReg(MDMCFG4,0x4B);    // Modem Configuration. Value from wixel project
  WriteReg(MDMCFG3,0x11);    // Modem Configuration. Value from wixel project
  WriteReg(MDMCFG2,0x73);    // Modem Configuration. Value from wixel project
  WriteReg(MDMCFG1,0x03);    // Modem Configuration. Value from wixel project
  WriteReg(MDMCFG0,0x55);    // Modem Configuration. Value from wixel project

  WriteReg(DEVIATN,0x00);

  WriteReg(FOCCFG,0x0A);     // Frequency Offset Compensation configuration. Value from wixel project
  
  WriteReg(FSCTRL1,0x0A);    // From wixel project
  WriteReg(FSCTRL0,0x00);

  WriteReg(PKTCTRL0,0x05);     // Variable packet length mode. Packet length configured by the first byte after sync word
                               // CRC calculation in TX and CRC check in RX enabled
  WriteReg(PKTCTRL1,0x04);     // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as
                               // well as the CRC OK flag.                      
                               
  WriteReg(MCSM0,0x14);      // Main Radio Control State Machine Configuration
  WriteReg(MCSM1,0x0D);      // After RX, stay RX.  After TX, go to FSTXON. In Wixel - 0x05;    Disable CCA.  After RX, go to FSTXON.  After TX, go to FSTXON.
  WriteReg(MCSM2,0x07);
  
  WriteReg(AGCCTRL2,0x44);   // AGC Control. Value from wixel project
  WriteReg(AGCCTRL1,0x00);   // AGC Control. Value from wixel project
  WriteReg(AGCCTRL0,0xB2);   // AGC Control. Value from wixel project
  
  WriteReg(FSCAL3,0xA9);     // Frequency Synthesizer Calibration. Value from wixel project
  WriteReg(FSCAL2,0x0A);     // Frequency Synthesizer Calibration. Value from wixel project 
  WriteReg(FSCAL1,0x20);     // Frequency Synthesizer Calibration. Value from wixel project
  WriteReg(FSCAL0,0x0D);     // Frequency Synthesizer Calibration. Value from wixel project

  WriteReg(TEST2,0x81);      // Value from wixel project
  WriteReg(TEST1,0x35);      // Value from wixel project
  WriteReg(TEST0,0x0B);      // Value from wixel project

  WriteReg(SYNC1,0xD3);      // Value from wixel project
  WriteReg(SYNC0,0x91);      // Value from wixel project
  
  WriteReg(ADDR,0x00);

  WriteReg(FREND1,0xB6);      // Value from wixel project
  WriteReg(FREND0,0x10);      // Value from wixel project

  WriteReg(BSCFG,0x6C);      // Value from wixel project

}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK
  digitalWrite(SS,HIGH);
#ifdef DEBUG
  Serial.println("CC250 init start ....");
#endif  
  init_CC2500();  // initialise CC2500 registers
#ifdef DEBUG
  Serial.println("CC250 init OK");
  byte i;
  i = ReadReg(PARTNUM);
  Serial.print("Part Number: ");
  Serial.println(i);
  i = ReadReg(VERSION);
  Serial.print("Version Number: ");
  Serial.println(i);
#endif  
  memset (&Pkt, 0, sizeof (Dexcom_packet));

  dex_tx_id= asciiToDexcomSrc(transmitter_id);
  
}

unsigned long delayFor(int wait_chan) {
    if(needsTimingCalibration) {
        return delayedWaitTimes[wait_chan];
    }
    if(!wait_chan && sequential_missed_packets) {
        return waitTimes[wait_chan] + (sequential_missed_packets * wake_earlier_for_next_miss * 2 * 1000);
    } else {
        return waitTimes[wait_chan];
    }
}

void reset_offsets() {
    int i;
    for(i=0; i<4; i++) {
        fOffset[i] = defaultfOffset[i];
    }
}

void swap_channel(unsigned long channel, byte newFSCTRL0) {

   SendStrobe(SIDLE);
   WriteReg(FSCTRL0,newFSCTRL0);
   WriteReg(CHANNR,channel);
   SendStrobe(SRX);  //RX
}

boolean WaitForPacket(unsigned int milliseconds, Dexcom_packet * pkt, byte channel)
{
    unsigned long start = millis();
    unsigned long six_minutes = 360000;
    boolean nRet = false;
    byte rxbytes;
    
    swap_channel(nChannels[channel], fOffset[channel]);

    while (!milliseconds || (millis() - start) < milliseconds) {
      blink_builtin_led_quarter();
      if(millis() - start > six_minutes) {
        break;
      }
      rxbytes = ReadReg(RXBYTES);
      if (rxbytes > 0) {
         fOffset[channel] += ReadReg(FREQEST);
#ifdef DEBUG
         Serial.print("Bytes received: ");
         Serial.println(rxbytes);     
#endif      
         return true;
      }
      delay(500);
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    return nRet;
}
boolean get_packet (Dexcom_packet * pPkt) {
  int nChannel = 0;
  for (nChannel = 0; nChannel < NUM_CHANNELS; nChannel++)
  {
#ifdef DEBUG
  Serial.print("Listen chanel: ");
  Serial.println(nChannel);
#endif
    if (WaitForPacket (delayFor(nChannel), pPkt, nChannel)) {
      needsTimingCalibration = 0;
      sequential_missed_packets = 0;
      delay(2000);
      SendStrobe(SIDLE);
      SendStrobe(SFRX);
      return true;
    } else
    {
      continue;
    }
   
  }
  sequential_missed_packets ++;
  if(sequential_missed_packets > misses_until_failure) {
      sequential_missed_packets = 0;
      needsTimingCalibration = 1;
  }
  reset_offsets();
  last_catch_channel = 0;
  SendStrobe(SIDLE);
  
  return false;
}

void print_packet(Dexcom_packet * pPkt) {
#ifdef DEBUG
  Serial.println("print_packet start .....");
#endif  
}

void loop() {
  if (get_packet (&Pkt))
  {
     print_packet (&Pkt);
  }

}



