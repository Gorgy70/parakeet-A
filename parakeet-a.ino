#define DEBUG

#include <SPI.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "cc2500_REG.h"

#define GDO0_PIN 2            // Цифровой канал, к которму подключен контакт GD0 платы CC2500
#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов
#define FIVE_MINUTE 300000    // 5 минут

#define my_webservice_url  "http://parakeet.esen.ru/receiver.cgi"
#define my_webservice_reply     "!ACK"
#define my_user_agent     "parakeet_A"
#define my_gprs_apn   "internet.mts.ru"
#define my_password_code  "12354"

unsigned long dex_tx_id;
//char transmitter_id[] = "ABCDE";
char transmitter_id[] = "6518Y";

unsigned long packet_received = 0;

byte fOffset[NUM_CHANNELS] = { 0x00, 0xD5, 0xE6, 0xE5 };
byte defaultfOffset[NUM_CHANNELS] = { 0x00, 0xD5, 0xE6, 0xE5 };
//byte fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
//byte defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
//unsigned long waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
unsigned long waitTimes[NUM_CHANNELS] = { 0, 500, 500, 500 };
//unsigned long waitTimes[NUM_CHANNELS] = { 0, 550, 550, 550 };
unsigned long catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };

byte sequential_missed_packets = 0;
byte wait_after_time = 100;
unsigned long next_time = 0; // Время ожидания следующего пакета на канале 0
unsigned long catch_time = 0; // Время последнего пойманного пакета (приведенное к пакету на канале 0)

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

typedef struct _parakeet_settings
{
  unsigned long dex_tx_id;     //4 bytes
  char http_url[56];
  char gsm_apn[32];
  char password_code[6];
  unsigned long checksum; // needs to be aligned

} parakeet_settings;

parakeet_settings settings;

char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                        };


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
  for (i = 0; i < 32; i++) {
    if (SrcNameTable[i] == srcVal) break;
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

void clearSettings()
{
  memset (&settings, 0, sizeof (settings));
  settings.dex_tx_id = asciiToDexcomSrc (transmitter_id);
  dex_tx_id = settings.dex_tx_id;
  sprintf(settings.http_url, my_webservice_url);
  sprintf(settings.gsm_apn, my_gprs_apn);
  sprintf(settings.password_code, my_password_code);
}

unsigned long checksum_settings()
{
  char* flash_pointer;
  unsigned long chk = 0x12345678;
  byte i;
  //   flash_pointer = (char*)settings;
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings) - 4; i++)
  {
    chk += (flash_pointer[i] * (i + 1));
    chk++;
  }
  return chk;
}

void saveSettingsToFlash()
{
  settings.checksum = checksum_settings();
  EEPROM.put(0, settings);
}

void loadSettingsFromFlash()
{
  EEPROM.get(0, settings);
  if (settings.checksum != checksum_settings()) {
#ifdef DEBUG
    Serial.println("Settings checksum error. Load defaults");
#endif
    clearSettings();
  }
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(settings.dex_tx_id);
#endif
}

void blink_builtin_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void blink_builtin_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void WriteReg(char addr, char value) {
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
  //  delay(10);
}

char SendStrobe(char strobe)
{
  digitalWrite(SS, LOW);

  while (digitalRead(MISO) == HIGH) {
  };

  char result =  SPI.transfer(strobe);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return result;
}

void init_CC2500() {

  SendStrobe(SRES);       // software reset for CC2500
  WriteReg(IOCFG0, 0x06);
  WriteReg(SYNC1, 0xD3);
  WriteReg(SYNC0, 0x91);

  WriteReg(PKTCTRL1, 0x0C); // CRC_AUTOFLUSH = 1 & APPEND_STATUS = 1
  //  WriteReg(PKTCTRL1,0x04);
  WriteReg(PKTCTRL0, 0x05);

  WriteReg(FSCTRL1, 0x08);
  WriteReg(FSCTRL0, 0x00);

  WriteReg(FREQ2, 0x5D);
  WriteReg(FREQ1, 0x44);
  WriteReg(FREQ0, 0xEB);

  WriteReg(MDMCFG4, 0x4A);
  WriteReg(MDMCFG3, 0xF8);
  WriteReg(MDMCFG2, 0x73);
  WriteReg(MDMCFG1, 0x03);
  WriteReg(MDMCFG0, 0x3B);

  WriteReg(DEVIATN, 0x00);

  WriteReg(MCSM0, 0x18);

  WriteReg(FOCCFG, 0x16);

  WriteReg(BSCFG, 0x6C);

  WriteReg(AGCCTRL2, 0x03);
  WriteReg(AGCCTRL1, 0x40);
  WriteReg(AGCCTRL0, 0x91);

  WriteReg(FREND1, 0x56);
  WriteReg(FREND0, 0x10);

  WriteReg(FSCAL3, 0xA9);
  WriteReg(FSCAL2, 0x0A);
  WriteReg(FSCAL1, 0x00);
  WriteReg(FSCAL0, 0x11);

  WriteReg(TEST2, 0x88);
  WriteReg(TEST1, 0x31);
  WriteReg(TEST0, 0x0B);

  WriteReg(MCSM0, 0x14);   // Auto-calibrate when going from idle to RX or TX.
  WriteReg(MCSM1, 0x00);   // Disable CCA.  After RX, go to IDLE.  After TX, go to IDLE.
}

char ReadReg(char addr) {
  addr = addr + 0x80;
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return y;
}

char ReadStatus(char addr) {
  addr = addr + 0xC0;
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return y;
}

void setup() {
  pinMode(GDO0_PIN, INPUT);
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  SPI.begin();
  //  SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK
  digitalWrite(SS, HIGH);

  init_CC2500();  // initialise CC2500 registers
  loadSettingsFromFlash();
}

void reset_offsets() {
  int i;
  for (i = 0; i < 4; i++) {
    fOffset[i] = defaultfOffset[i];
  }
}

void swap_channel(unsigned long channel, byte newFSCTRL0) {

  SendStrobe(SIDLE);
  //   WriteReg(FSCTRL0,newFSCTRL0);
  WriteReg(CHANNR, channel);
  SendStrobe(SRX);  //RX
  while (ReadStatus(MARCSTATE) != 0x0d) {
    // Подождем пока включится режим приема
  }
}

void ReadRadioBuffer() {
  char buffer[64];
  byte len;
  byte i;
  byte rxbytes;

  memset (&buffer, 0, sizeof (Dexcom_packet));
  len = ReadStatus(RXBYTES);
#ifdef DEBUG
  Serial.print("Bytes in buffer: ");
  Serial.println(len);
#endif
  if (len > 0 && len < 65) {
    for (i = 0; i < len; i++) {
      if (i < sizeof (Dexcom_packet)) {
        buffer[i] = ReadReg(RXFIFO);
      }
    }
  }
  memcpy(&Pkt, &buffer, sizeof (Dexcom_packet));
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(Pkt.src_addr);
#endif
}

boolean WaitForPacket(unsigned int milliseconds_wait, byte channel_index)
{
  unsigned long start_time;
  unsigned long current_time;
  boolean nRet = false;
  boolean packet_on_board;

  start_time = millis();
  swap_channel(nChannels[channel_index], fOffset[channel_index]);

#ifdef DEBUG
  Serial.print("Chanel = ");
  Serial.print(nChannels[channel_index]);
  Serial.print(" Time = ");
  Serial.println(start_time);
#endif
  while (true) {
    current_time = millis();
    if (milliseconds_wait != 0 && current_time - start_time > milliseconds_wait) {
      break; // Если превысыли время ожидания на канале - выход
    }
    if (channel_index == 0 && next_time != 0 && current_time > next_time + wait_after_time) {
      break; // Если превысыли время следующего пакета на канале 0 - выход
    }
    blink_builtin_led_quarter();
    packet_on_board = false;
    while (digitalRead(GDO0_PIN) == HIGH) {
      packet_on_board = true;
      // Идет прием пакета
    }
    if (packet_on_board) {
      fOffset[channel_index] = ReadStatus(FREQEST);
      ReadRadioBuffer();
      if (Pkt.src_addr == dex_tx_id) {
#ifdef DEBUG
        Serial.print("Packet catched. Chanel = ");
        Serial.print(nChannels[channel_index]);
        Serial.print(" Interval = ");
        if (catch_time != 0) {
          Serial.println(current_time - 500 * channel_index - catch_time);
        }
        else {
          Serial.println("unknown");
        }
#endif
        catch_time = current_time - 500 * channel_index; // Приводим к каналу 0
        nRet = true;
      }
      if (next_time != 0 && !nRet && channel_index == 0 && current_time < next_time && next_time-current_time < 2000) {
#ifdef DEBUG
        Serial.print("Chanel = 0. Second try.");
        Serial.print(nChannels[channel_index]);
        Serial.print(" Time = ");
        Serial.println(current_time);
#endif
        swap_channel(nChannels[channel_index], fOffset[channel_index]);
      }
      else {
        break;
      }
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  return nRet;
}

boolean get_packet (void) {
  byte nChannel;
  boolean nRet;

  nRet = false;
  for (nChannel = 0; nChannel < NUM_CHANNELS; nChannel++)
  {
    if (WaitForPacket (waitTimes[nChannel], nChannel)) {
      nRet = true;
      break;
    }
  }
  if (!nRet) {
    sequential_missed_packets++;
#ifdef DEBUG
    Serial.print("Packet missed - ");
    Serial.println(sequential_missed_packets);
#endif
    if (sequential_missed_packets > misses_until_failure) { // Кол-во непойманных пакетов превысило заданное кол-во. Будем ловить пакеты непрерывно
      next_time = 0;
    }
  }
  else {
    sequential_missed_packets = 0; // Сбрасываем счетчик непойманных пакетов
    next_time = catch_time; 
  }

  if (next_time != 0) {
    next_time += FIVE_MINUTE;
  }
  SendStrobe(SIDLE);
  SendStrobe(SFRX);

  return nRet;
}

void print_packet() {
  byte i;
#ifdef DEBUG
  Serial.print(Pkt.len, HEX);
  Serial.print("\t");
  Serial.print(Pkt.dest_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.src_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.port, HEX);
  Serial.print("\t");
  Serial.print(Pkt.device_info, HEX);
  Serial.print("\t");
  Serial.print(Pkt.txId, HEX);
  Serial.print("\t");
  Serial.print(Pkt.raw, HEX);
  Serial.print("\t");
  Serial.print(Pkt.filtered, HEX);
  Serial.print("\t");
  Serial.print(Pkt.battery, HEX);
  Serial.print("\t");
  Serial.print(Pkt.unknown, HEX);
  Serial.print("\t");
  Serial.print(Pkt.checksum, HEX);
  Serial.print("\t");
  Serial.print(Pkt.RSSI, HEX);
  Serial.print("\t");
  Serial.print(Pkt.LQI2, HEX);
  Serial.println(" OK");
#endif
}

void loop() {
  unsigned long current_time;
  
  if (next_time != 0) {
#ifdef DEBUG
    Serial.print("next_time - ");
    Serial.print(next_time);
    Serial.print(" current_time - ");
    Serial.print(millis());
    Serial.print(" interval - ");
    Serial.println(next_time - millis() - 3000);
#endif
    current_time = millis();
    if  (next_time > current_time && (next_time - current_time) < FIVE_MINUTE)  {
      delay(next_time - current_time - 2000); // Можно спать до следующего пакета. С режимом сна будем разбираться позже
#ifdef DEBUG
      Serial.println("WakeUp");
#endif
    }
    else {
#ifdef DEBUG
      Serial.println("Timer overflow");
#endif
      next_time = 0;
    }
  }
  if (get_packet ())
  {
    print_packet ();
  }

}



