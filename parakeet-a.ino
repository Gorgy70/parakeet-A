#define DEBUG
#define GSM-MODEM

#include <SPI.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "cc2500_REG.h"

#define GDO0_PIN 2            // Цифровой канал, к которму подключен контакт GD0 платы CC2500
#define DTR_PIN  5            // Цифровой канал, к которму подключен контакт DTR платы GSM-модема
#define RX_PIN   8            // Rx контакт для последовательного порта
#define TX_PIN   9            // Tx контакт для последовательного порта
#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов
#define FIVE_MINUTE 300000    // 5 минут

#define my_webservice_url  "http://parakeet.esen.ru/receiver.cgi"
#define my_webservice_reply     "!ACK"
#define my_user_agent     "parakeet_A"
#define my_gprs_apn   "internet.mts.ru"
#define my_password_code  "12354"

SoftwareSerial mySerial(RX_PIN, TX_PIN); // RX, TX

unsigned long dex_tx_id;
//char transmitter_id[] = "ABCDE";
char transmitter_id[] = "6518Y";

unsigned long packet_received = 0;

byte fOffset[NUM_CHANNELS] = { 0x00, 0xD5, 0xE6, 0xE5 };
//byte defaultfOffset[NUM_CHANNELS] = { 0x00, 0xD5, 0xE6, 0xE5 };
//byte fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
//byte defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
//unsigned long waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
//unsigned long waitTimes[NUM_CHANNELS] = { 0, 500, 500, 500 };
unsigned long waitTimes[NUM_CHANNELS] = { 0, 600, 600, 600 };
//unsigned long catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };

byte sequential_missed_packets = 0;
byte wait_after_time = 100;
unsigned long next_time = 0; // Время ожидания следующего пакета на канале 0
unsigned long catch_time = 0; // Время последнего пойманного пакета (приведенное к пакету на канале 0)

byte misses_until_failure = 2;                                                   //
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //

boolean gsm_availible = false; // Доступность связи GSM
char SerialBuffer[190] ; // Буффер для чтения данных их последовательного порта

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

byte bit_reverse_byte (byte in)
{
    byte bRet = 0;
    if (in & 0x01)
        bRet |= 0x80;
    if (in & 0x02)
        bRet |= 0x40;
    if (in & 0x04)
        bRet |= 0x20;
    if (in & 0x08)
        bRet |= 0x10;
    if (in & 0x10)
        bRet |= 0x08;
    if (in & 0x20)
        bRet |= 0x04;
    if (in & 0x40)
        bRet |= 0x02;
    if (in & 0x80)
        bRet |= 0x01;
    return bRet;
}

void bit_reverse_bytes (byte * buf, byte nLen)
{
    byte i = 0;
    for (; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte (buf[i]);
    }
}

unsigned long dex_num_decoder (unsigned int usShortFloat)
{
    unsigned int usReversed = usShortFloat;
    byte usExponent = 0;
    unsigned long usMantissa = 0;
    bit_reverse_bytes ((byte *) & usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
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

#ifdef GSM-MODEM
boolean gsm_command(const char *command, const char *response, int timeout) {
  boolean ret;
  unsigned long timeout_time; 
  int len = strlen (response);
  int loop = 0;
  
  digitalWrite(LED_BUILTIN, HIGH);

  if (len == 0) {
    ret = true;
  } 
  else {
    ret - false;
  }  
  memset (&SerialBuffer,0,sizeof(SerialBuffer));
  mySerial.write(command);
  mySerial.write("\r\n"); // enter key
  timeout_time = millis() + timeout * 1000;
  while (millis() < timeout_time)
  {
    if (mySerial.available()) {
      SerialBuffer[loop] = mySerial.read();
      loop++;
      if (loop > len) {
        if (strncmp(command,SerialBuffer[loop-len],len) == 0) {
          ret = true;
          break;
        }
      }  
    } 
    else {
      delayMicroseconds(100);
    }
  }
#ifdef DEBUG
  Serial.print("GSM Command = ");
  Serial.println(command);
  Serial.print("Expected response = ");
  Serial.println(response);
  Serial.print("Real response = ");
  Serial.println(SerialBuffer);
  Serial.print("Result = ");
  Serial.println(ret);
#endif
  digitalWrite(LED_BUILTIN, LOW);
  return ret;
}

boolean set_gprs_profile() {
  boolean ret;
  char    cmd[40];
  
  gsm_command("AT+SAPBR=0,1", "OK", 3); // Сбросим настроенный GPRS профиль
  ret = gsm_command("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2); // Настраиваем мобильный интернет 2G
  if (ret) {
    sprintf(cmd,"AT+SAPBR=3,1,\"APN\",\"%s\"",settings.gsm_apn);   // Точка доступа
    ret = gsm_command(cmd,"OK",2);
    if (ret) {
      ret = gsm_command("AT+SAPBR=1,1", "OK", 30); // Применяем настройки
    }
  }
  return ret;    
}

void read_sms() {
  boolean ret;

  gsm_command("AT+CMGL=\"REC UNREAD\"" ,"OK",5); // Читаем все новые смс-ки в буфер
// Здесь надо разбирать принятый буфер, читать команды  и отправлять ответы
  gsm_command("AT+CMGDA=\"DEL READ\"","OK",5); // Удалить прочитанные смс-ки
  gsm_command("AT+CMGDA=\"DEL SENT\"","OK",5); // Удалить отправленные смс-ки
}

void gsm_goto_sleep() {
  gsm_command("AT+CSCLK=1", "OK", 2); // Переводим модем в режим сна в режиме управления сигналом DTR
  digitalWrite(DTR_PIN, HIGH);
}

void init_GSM() {
  gsm_command("ATZ","OK",10); // Установить параметры по умолчанию 
  gsm_command("ATE0","OK", 2); // Выключить эхо 
  gsm_availible = gsm_command("AT+CFUN=1", "Call Ready", 30); // Подключаемся к сети  
  
  if (gsm_availible) {
    gsm_command("AT+CMGF=1","OK",2); // Устанавливаем текстовый режим чтения смс
    read_sms();
    set_gprs_profile();
    gsm_goto_sleep();
  }  
}

void gsm_get_location(char *location) {
  unsigned long longitudeMinor;
  unsigned long latitudeMinor;
  unsigned int  longitudeMajor;
  char minus[2]="-";
  byte latitudeMajor;

  location[0]='\0';
  if (gsm_command("AT+CIPGSMLOC=1,1","OK",15)) {
    if (strlen(SerialBuffer)>12){
      sscanf(&SerialBuffer[12],"%d.%D,%h.%D,",&longitudeMajor,&longitudeMinor,&latitudeMajor,&latitudeMinor);
      if ((longitudeMajor==0)&&(SerialBuffer[12]=='-')) {
         minus[0]='-';
      }
      else
      {
        minus[0]='\0';
      }
      sprintf(location,"%hhd.%ld,%s%d.%ld",latitudeMajor,latitudeMinor,minus,longitudeMajor,longitudeMinor);
//      if ((longitudeMajor==0)&&(captureBuffer[2]=='-')) longitudeMajor=255;
    }
  }
}

void gsm_get_battery(byte *percent,int *millivolts) {
  byte charging;  

  percent = 0;
  millivolts = 0;
  if (gsm_command("AT+CBC","OK",2)) {
    sscanf(SerialBuffer[6],"%h,%h,%d",&charging,&percent,&millivolts);
  }  
}

#endif

void setup() {
  pinMode(GDO0_PIN, INPUT);
  pinMode(DTR_PIN, OUTPUT);
  analogReference(INTERNAL);
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
  mySerial.begin(9600);
  loadSettingsFromFlash(); 
#ifdef GSM-MODEM
  init_GSM();
#endif
}

/*
void reset_offsets() {
  int i;
  for (i = 0; i < 4; i++) {
    fOffset[i] = defaultfOffset[i];
  }
}
*/
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
  char gsm_cmd[180];
  char params[128];
  char lastLocation[30];  
  byte batteryPercent = 0;
  byte batteryCharging;
  int batteryMillivolts = 0;
  
#ifdef GSM-MODEM
  digitalWrite(DTR_PIN, LOW); // Будим GSM-модем

  gsm_command("AT+HTTPTERM", "OK", 2); // Завершить сессию на вскяий случай
  gsm_command("AT+HTTPINIT", "OK", 10); // Начинаем http сессию
  gsm_command("AT+HTTPPARA=\"CID\",1", "OK", 2) ;  

  gsm_get_location(lastLocation);
  gsm_get_battery(batteryPercent, batteryMillivolts);
// Адрес сервера паракита
  sprintf(params,"?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d&gl=%s",millis(),dex_tx_id,settings.password_code,dex_num_decoder(Pkt.raw),
                                                                                      dex_num_decoder(Pkt.filtered)*2,Pkt.battery,millis()-catch_time,
                                                                                      batteryPercent, batteryMillivolts, analogRead(8)-290, lastLocation);         
  sprintf(gsm_cmd,"AT+HTTPPARA=\"URL\",\"%s%s\" ",settings.http_url,params);

  gsm_command(gsm_cmd,"OK",2) ;

  gsm_command("AT+HTTPPARA=\"UA\",\"" my_user_agent "\"", "OK", 2);  // User agent для http запроса
  gsm_command("AT+HTTPACTION=0", "+HTTPACTION: 0,200,", 60) ; // Отправляем запрос на сервер
  gsm_command("AT+HTTPREAD", my_webservice_reply , 2) ; // Читаем ответ вэб-сервиса
  gsm_command("AT+HTTPTERM", "OK", 2); // Завершаем http сессию
#endif
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
  Serial.print(dex_num_decoder(Pkt.raw));
  Serial.print("\t");
  Serial.print(dex_num_decoder(Pkt.filtered)*2);
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
#ifdef GSM-MODEM
  read_sms(); // Прочитаем полученные смс-ки
#endif

}



