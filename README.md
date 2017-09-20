# parakeet-A

Перевод проекта Parakeet с платформы Wixel на платофрму Arduino.<br>
Данный проект основан на следующих проектах:<br>
1. Parakeet от Jamorham (https://jamorham.github.io, https://github.com/jamorham/wixel-xDrip)<br>
2. XDrip от Emma Black (http://stephenblackwasalreadytaken.github.io/xDrip, https://github.com/StephenBlackWasAlreadyTaken/xDrip)<br>
3. CC2500-Project от Don Browne (https://github.com/brownedon/CC2500-Project)<br>
<br>
<br>
<b>Сборка прибора:</b><br>
<br>
Для сборки прибора необходимы следующие компоненты:<br>
1. Контроллер Arduino Mini Pro ATmega328P 3.3V, 8MHz. Важно иметь именно такой контроллер. У котроллера на чипе ATmega168PA мало памяти, а контроллеры с напряжением 5V не будут работать.<br>
2. Радиомодуль на базе чипа Texas Instruments CC2500, желательно с усилителем слабого сигнала.<br>
Таких модулей не очень много в продаже, я их покупал на али у продавца e_goto Processors Store (https://www.aliexpress.com/store/1829717?spm=2114.13010608.0.0.rIN6UB).<br>
Были куплены и испытаны два модуля:<br>
https://www.aliexpress.com/item/Wireless-Module-CC2500-2-4G-Low-power-Consistency-Stability-Small-Size/32702148262.html?spm=2114.13010608.0.0.Y8KBPk<br>
Данный модуль без усилителя слабого сигнала обладает слабой чувствительностью и принимает сигнал на расстояниее не более 5 метров при отстутсвии препятсвий. Модуль не принимает сигнал через стену.<br>
https://www.aliexpress.com/item/CC2500-PA-LNA-2-4G-SPI-22dBm-Wireless-Data-Transceiver-Module/32606419424.html?spm=2114.13010608.0.0.X1eJmz<br>
Это модуль имеет усилитель слабого сигнала и очень неплохую чувствительность и рекомендуется к покупке.<br>
3. Модуль контроля заряда батареи.<br>
4. Аккумулятор 3.7 В.<br>
Я нашел аккумулятор под размер печатной платы:<br>
https://www.aliexpress.com/item/754060-MP3-MP4-2500MAH-3-7V-Bluetooth-stereo-mobile-power-polymer-lithium-battery/32790808657.html?spm=2114.13010608.0.0.TSSHRb
<br>
но можно использовать любой подходящий.<br>
5. Адаптер USB-Serial для заливки прошивки. Я покупал вот этот: http://robotdyn.ru/catalog/boards/usb_serial_adapter_ch340g_5v_3_3v/<br>
<br>
<br>
Соединяем между собой модули:<br>
VCC Радиомодуля - Контакт 3.3V Ардуино<br>
SCLK Радиомодуля - Контакт 13 Ардуино<br>
SI Радиомодуля - Контакт 11 Ардуино<br>
SO Радиомодуля - Контакт 12 Ардуино<br>
CSN Радиомодуля - Контакт 10 Ардуино<br>
GDO Радиомодуля - Контакт 9 Ардуино (может быть любым цифровым контактом. Назначается строкой #define GDO0_PIN в прошивке)<br>
GND Радиомодуля - Контакт GND Ардуино (земля)<br>
LEN Радиомодуля - Соединяется или с VCC Радиомодуля или контактом А1 Ардуино. Соединяется через сопротивление 10КОм. В случае подключения на контакт А1, необходимо компилировать скетч с ключом CC2500_LEN_CONTROL<br>
DTR GSM-Модема - Контакт 8 Ардуино (может быть любым цифровым контактом. Назначается строкой #define DTR_PIN в прошивке)<br>
RX GSM-Модема - Контакт 7 Ардуино (может быть любым цифровым контактом. Назначается строкой #define TX_PIN в прошивке)<br>
TX GSM-Модема - Контакт 6 Ардуино (может быть любым цифровым контактом. Назначается строкой #define RX_PIN в прошивке)<br>
GND GSM-Модема - На минус платы зарядки<br>
VCC GSM-Модема - На плюс платы зарядки<br>
Минус платы зарядки - Контакт GND Ардуино (земля)<br>
Контакт 5 Ардуино (может быть любым цифровым контактом. Назначается строкой #define YELLOW_LED_PIN в прошивке) - на желтый светодиод. Подключается последовательно с сопротивлением 100 Ом на землю.<br>
Контакт 4 Ардуино (может быть любым цифровым контактом. Назначается строкой #define RED_LED_PIN в прошивке) - на красный светодиод. Подключается последовательно с сопротивлением 100 Ом на землю.<br>
<br>
Для заливки прошивки в контроллер подключаем адаптер USB-Serial следующим образом:<br>
RX адаптера USB-Serial - RX Ардуино<br>
TX адаптера USB-Serial - TX Ардуино<br>
GND адаптера USB-Serial - GND Ардуино<br>
DTR адаптера USB-Serial - DTR Ардуино (можно подключить к контакту RST Ардуино через конденсатор в 1 мкФ<br>
Данная схема подключения подходит для моего адаптера USB-Serial. Для других может потребоваться другое подключение:br>
TX адаптера USB-Serial - RX Ардуино<br>
RX адаптера USB-Serial - TX Ардуино<br>
Правильная схема подключения адаптера выбирается экспериментально.<br>
<br>
В файле parakeet-A.lay6 находится печатная плата прибора в формате Sprint-Layout 6 (выражаю огромную благодарность Олегу Романовскому за помощь в разработке) <br>
К сожалению, в этой схеме не отражено подключение контакта LEN через сопротивление, а потому рекомендуется разорвать дорожку между контктами VCC и LEN радиомодуля и припаять сопротивление 10КОМ между контактом LEN радиомодуля и А1 Ардуины.<br>
<br>
<b>Заливка прошивки в прибор:</b><br>
<br>
Для заливки прошивки необходимо установить Arduino IDE:<br>
https://www.arduino.cc/en/Main/Software
<br>
Открыть проект Parakeet-A, выбрать плату "Arduino Pro or Pro Mini", выбрать процессор "ATmega 328 (3.3V, 8MHz)", выбрать порт, скомпилировать и загрузить проект.<br>
В исходном тексте проекта можно вносить следующие изменения:<br>
ИД трансмиттера в строке<br>
char transmitter_id[] = "ABCDE";<br>
Так же ИД трансмиттера можно изменить с помощью СМС с текстом TRANSMIT <ИД трансмиттера><br>
APN вашего провайдера мобильной связи в строке<br>
#define my_gprs_apn   "internet.mts.ru"<br>
Этот параметр также можно изменить с помощью СМС с текстом APN <APN><br>
Цифровой код, который необходим для получения данных в программе xDrip. Цифровой код можно поменять в строке<br>
#define my_password_code  "12354"<br>
или с помощью СМС с текстом PWD <цифровой код>. Цифровой код должен состоять из 5 цифр.<br>
Адрес облачного сервиса. Адрес облачного сервиса, указанный в исходном коде прошивки является рабочим и абсолютно бесплатным, но вы можете указать свой собственный облачные сервис в строке<br>
#define my_webservice_url  "http://parakeet.esen.ru/receiver.cgi"<br>
или с помощью СМС с текстом HTTP <адрес облачного сервиса><br>
