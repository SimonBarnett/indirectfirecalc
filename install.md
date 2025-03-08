# Installing Code on Arduino Nano for RF-Minimized Spotter System 

[Українська версія](https://github.com/SimonBarnett/indirectfirecalc/blob/main/install.md#%D0%B2%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BB%D0%B5%D0%BD%D0%BD%D1%8F-%D0%BA%D0%BE%D0%B4%D1%83-%D0%BD%D0%B0-arduino-nano-%D0%B4%D0%BB%D1%8F-%D1%81%D0%B8%D1%81%D1%82%D0%B5%D0%BC%D0%B8-%D0%BF%D1%80%D0%B8%D1%86%D1%96%D0%BB%D1%8E%D0%B2%D0%B0%D0%BD%D0%BD%D1%8F-%D0%B7-%D0%BC%D1%96%D0%BD%D1%96%D0%BC%D0%B0%D0%BB%D1%8C%D0%BD%D0%B8%D0%BC-%D1%80%D0%B0%D0%B4%D1%96%D0%BE%D0%B2%D0%B8%D0%BF%D1%80%D0%BE%D0%BC%D1%96%D0%BD%D1%8E%D0%B2%D0%B0%D0%BD%D0%BD%D1%8F%D0%BC)

This guide outlines the steps to upload code to an Arduino Nano for the RF-Minimized Weapon-Mounted Spotter System devices (S1/S2, B1, W1) using the Arduino-IDE.

## Prerequisites

### Hardware
- **Arduino Nano**: One for each device (S1/S2, B1, W1).
- **USB Mini-B Cable**: For connecting the Nano to your computer.
- **Computer**: Windows, macOS, or Linux.

### Software
- **Arduino IDE**: Download and install from [arduino.cc](https://www.arduino.cc/en/software) (version 1.8.x or 2.x).
- **Drivers**: For Nano clones with CH340/CH341 chipset, install the CH340 driver:
  - Windows/macOS: Search "CH340 driver" and follow vendor instructions.
  - Linux: Typically included in modern kernels; check with `lsusb` if needed.

### Libraries
Install these via Arduino IDE Library Manager (`Sketch > Include Library > Manage Libraries`):
- **Adafruit_HMC5883_U**: For compass (S1/S2, W1).
- **Adafruit_BME280**: For environmental sensor (W1).
- **Adafruit_GFX** and **Adafruit_SSD1306**: For OLED (B1).
- **RTClib** by Adafruit: For real-time clock (S1/S2, B1).
- **SoftwareSerial**: Usually pre-installed with Arduino IDE.

## Steps to Install Code

1. **Prepare Your Environment**
   - Open the Arduino IDE.
   - Connect the Arduino Nano to your computer using the USB Mini-B cable.
   - Verify the Nano’s power LED lights up.

2. **Select the Board and Port**
   - Go to `Tools > Board > Arduino AVR Boards > Arduino Nano`.
   - For clones with ATmega328P Old Bootloader, select `Tools > Processor > ATmega328P (Old Bootloader)`; otherwise, use default.
   - Go to `Tools > Port` and select the Nano’s port:
     - Windows: e.g., `COM3` or `COM4`.
     - macOS/Linux: e.g., `/dev/ttyUSB0` or `/dev/cu.usbserial-XXXX`.
   - If no port appears, check cable, drivers, or USB port.

3. **Load the Code**
   - Copy the relevant sketch (S1/S2, B1, or W1) from the project documentation.
   - Paste into a new Arduino IDE sketch (`File > New`, replace default code).
   - For S1/S2, set `DEVICE_ID` to `1` (S1) or `2` (S2).

4. **Verify the Code**
   - Click `Verify` (checkmark icon) or `Sketch > Verify/Compile`.
   - Check the output window for errors and resolve any missing library issues.

5. **Upload the Code**
   - Click `Upload` (right arrow icon) or `Sketch > Upload`.
   - The IDE compiles and uploads; Nano’s TX/RX LEDs will blink.
   - Confirm “Done uploading” in the output window.

6. **Test the Upload**
   - Open Serial Monitor (`Tools > Serial Monitor` or Ctrl+Shift+M), set baud rate to `9600`.
   - **S1/S2**: Press trigger button; check LED (green flash on success, red on failure) or serial output.
   - **B1**: Verify OLED shows “Base Station Ready”; connect W1 for environmental data.
   - **W1**: Confirm green startup flash and serial environmental data.

## Troubleshooting

- **“Programmer not responding” Error**:
  - Confirm correct port and board selection.
  - Test USB cable; try “Old Bootloader” option for clones.
- **No Port Visible**:
  - Install CH340/CH341 drivers for clones; restart IDE/computer.
- **Library Errors**:
  - Install missing libraries via `Manage Libraries` or `.zip` files (`Sketch > Include Library > Add .ZIP Library`).
- **LEDs Not Responding**:
  - Verify wiring matches code pin assignments (e.g., D8 red, D9 green).

## Repeating for Each Device

- **S1 and S2**: Repeat, setting `DEVICE_ID` to `1` (S1) or `2` (S2).
- **B1**: Use B1 code, ensuring OLED and switches are connected.
- **W1**: Use W1 code, connecting BME280 and HMC5883L.

After uploading, assemble devices per hardware instructions, connect (e.g., W1 to B1 via USB), and test functionality over the 5km NLOS range.

For further issues, refer to the project documentation or Arduino forums.

# Встановлення коду на Arduino Nano для системи прицілювання з мінімальним радіовипромінюванням

Цей посібник описує кроки для завантаження коду на Arduino Nano для пристроїв системи прицілювання з мінімальним радіовипромінюванням (S1/S2, B1, W1) за допомогою Arduino IDE, що дозволяє прицілювання на відстані 5 км поза прямою видимістю (NLOS), що відповідає діапазону 81-мм міномета (~5.6 км).

## Передумови

### Апаратне забезпечення
- **Arduino Nano**: Один для кожного пристрою (S1/S2, B1, W1).
- **Кабель USB Mini-B**: Для підключення Nano до комп'ютера.
- **Комп'ютер**: Windows, macOS або Linux.

### Програмне забезпечення
- **Arduino IDE**: Завантажте та встановіть з [arduino.cc](https://www.arduino.cc/en/software) (версія 1.8.x або 2.x).
- **Драйвери**: Для клонів Nano з чипсетом CH340/CH341 встановіть драйвер CH340:
  - Windows/macOS: Знайдіть "драйвер CH340" в Інтернеті та дотримуйтесь інструкцій.
  - Linux: Зазвичай вбудовано в сучасні ядра; перевірте за допомогою `lsusb`, якщо потрібно.

### Бібліотеки
Встановіть через менеджер бібліотек Arduino IDE (`Sketch > Include Library > Manage Libraries`):
- **Adafruit_HMC5883_U**: Для компаса (S1/S2, W1).
- **Adafruit_BME280**: Для екологічного датчика (W1).
- **Adafruit_GFX** та **Adafruit_SSD1306**: Для OLED-дисплея (B1).
- **RTClib** (Adafruit): Для годинника реального часу (S1/S2, B1).
- **SoftwareSerial**: Зазвичай попередньо встановлено в Arduino IDE.

## Кроки встановлення

1. **Встановлення Arduino IDE**
   - Завантажте та встановіть з [arduino.cc](https://www.arduino.cc/en/software).
   - Переконайтеся, що бібліотеки встановлені через менеджер бібліотек.

2. **Підключення Nano**
   - Підключіть Arduino Nano до комп'ютера за допомогою кабелю USB Mini-B.
   - Переконайтеся, що світлодіод живлення Nano світиться.

3. **Налаштування IDE**
   - Відкрийте Arduino IDE.
   - Виберіть `Tools > Board > Arduino AVR Boards > Arduino Nano`.
   - Для клонів з ATmega328P Old Bootloader виберіть `Tools > Processor > ATmega328P (Old Bootloader)`, якщо потрібно; інакше залиште за замовчуванням.
   - Виберіть порт через `Tools > Port`:
     - Windows: наприклад, `COM3` або `COM4`.
     - macOS/Linux: наприклад, `/dev/ttyUSB0` або `/dev/cu.usbserial-XXXX`.
   - Якщо порт не відображається, перевірте кабель, драйвери або USB-порт.

4. **Завантаження коду**
   - Скопіюйте відповідний скетч (S1/S2, B1 або W1) із документації проекту або репозиторію.
   - Вставте в новий скетч у Arduino IDE (`File > New`, замініть стандартний код).
   - Для S1/S2 встановіть `DEVICE_ID` на `1` (S1) або `2` (S2).

5. **Перевірка та завантаження**
   - Натисніть `Verify` (значок галочки) для компіляції; виправте помилки (наприклад, відсутність бібліотек).
   - Натисніть `Upload` (значок стрілки вправо); спостерігайте за миготінням світлодіодів TX/RX на Nano.
   - Переконайтеся, що в нижньому вікні з’явиться повідомлення “Done uploading”.

6. **Тестування пристрою**
   - Відкрийте Serial Monitor (`Tools > Serial Monitor` або Ctrl+Shift+M), встановіть швидкість передачі `9600`.
   - **S1/S2**: Натисніть кнопку запуску; очікуйте зелений спалах світлодіода (успіх) або червоний (невдача), перевірте вивід у послідовному порту.
   - **B1**: Перевірте, чи OLED відображає “Base Station Ready”; підключіть W1 для перегляду екологічних даних.
   - **W1**: Переконайтеся, що зелений світлодіод спалахує раз при запуску, якщо датчики в нормі, червоний — при несправності; перевірте дані в послідовному порту.

## Усунення несправностей

- **Помилка “Programmer not responding”**:
  - Перевірте налаштування порту та плати; протестуйте кабель; спробуйте “Old Bootloader” для клонів.
- **Порт не видно**:
  - Встановіть драйвер CH340 для клонів; перезапустіть IDE/комп'ютер.
- **Помилки бібліотек**:
  - Встановіть відсутні бібліотеки через `Manage Libraries` або завантажте `.zip` файли (`Sketch > Include Library > Add .ZIP Library`).
- **Світлодіоди не працюють**:
  - Перевірте підключення відповідно до пінів у коді (D8 червоний, D9 зелений для S1/S2/B1; D2 червоний, D3 зелений для W1).

## Примітки для кожного пристрою

- **S1 та S2**: Повторіть процес, встановивши `DEVICE_ID` на `1` (S1) або `2` (S2).
- **B1**: Використовуйте код для B1, переконайтесь, що OLED та перемикачі підключені.
- **W1**: Використовуйте код для W1, підключивши BME280 та HMC5883L.

Після завантаження зберіть пристрої за інструкціями з апаратного забезпечення, підключіть (наприклад, W1 до B1 через USB) і протестуйте функціональність на відстані 5 км NLOS. Зверніться до документації проекту для повних деталей.