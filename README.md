# parakeet-A

Перевод проекта Parakeet с платформы Wixel на платофрму Arduino.<br>
Данный проект основан на следующих проектах:<br>
1. Parakeet от Jamorham (https://jamorham.github.io/,https://github.com/jamorham/wixel-xDrip)<br>
2. XDrip от Emma Black (http://stephenblackwasalreadytaken.github.io/xDrip/,https://github.com/StephenBlackWasAlreadyTaken/xDrip)<br>
3. CC2500-Project от Don Browne (https://github.com/brownedon/CC2500-Project)<br>
<br>
<br>
Сборка прибора:<br>
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


