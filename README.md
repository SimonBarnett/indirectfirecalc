# RF-Minimized Weapon-Mounted Spotter System for Multi-Weapon Indirect Fire Targeting

## Introduction

In Ukraine’s GPS-denied, DF-threatened environment as of March 1, 2025, Russian jamming disrupts traditional positioning, while RF direction finding (DF) endangers units emitting predictable signals.

This concept enables 5km non-line-of-sight (NLOS) targeting without GPS using time-of-flight (TOF) ranging, minimizing RF transmissions to evade DF detection. The 5km NLOS range aligns with the typical maximum range of an 81mm mortar (e.g., M252, ~5.6km with standard rounds).

Spotters initiate brief LoRa requests, the base computes fire missions from a ~70–140ms data burst, recalculating solutions for an 81mm mortar, 155mm artillery, or 120mm tank based on a tri-state switch and current or last-known environmental data.

Materials for the base system (base station and environmental sensor) costs **$115 to $125**, with each additional spotter unit costing **$69 to ~$74**.

## System Overview

- **Spotter Devices (S1, S2)**: Weapon-mounted, feature a power on/off button and red/green bi-colour LED (constant red on sensor failure, red flashes 1 second on failed fire mission transmission, green flashes 1 second on successful B1 receipt, single green flash on startup if sensors operational), request timestamp from base station on click, calculate TOF distance, send target data via LoRa (~70–140ms RF per mission) within a 5km NLOS range.
- **Base Station (B1)**: Responds to S1/S2 requests (~20–40ms TX over a 5km NLOS range), maintains a rolling list of 10 missions, recalculates all missions (charge, azimuth, range, elevation) with current environmental data or last-known data if unavailable, displays on a 0.96” OLED with up/down rocker switch scrolling and tri-state switch for weapon selection (81mm mortar, 155mm artillery, 120mm tank), features a red/green LED for W1 connection status (green = connected, red = disconnected), powers W1 via USB, runs on a 2000mAh battery for 12+ hours.
- **Environmental Sensor (W1)**: Measures wind speed, direction, temperature, humidity, pressure, USB-powered from B1, sends data without RF emissions, includes a red/green bi-colour LED (constant red on sensor failure, single green flash on startup if sensors operational).

## Spotter Device (S1, S2)

### Parts List

- **Microcontroller**: Arduino Nano ($5)
- **RF Transceiver**: Ebyte E32-433T20D (LoRa SX1276) ($15)
- **Compass**: HMC5883L ($3)
- **Real-Time Clock**: DS3231 RTC Module ($3)
- **Laser Rangefinder**: Wosports H-100 or similar ($20-$30)
- **Battery**: 3.7V 1000mAh LiPo ($6)
- **Antenna**: 433 MHz Whip ($3, if not bundled)
- **Trigger Button**: Tactile Switch ($1) - Initiates targeting request
- **Power Button**: Mini Slide Switch ($1) - On/off power control
- **Red/Green LED**: Bi-Color 5mm LED with 220Ω Resistors ($2) - Red for sensor failure or failed transmit, green for success/startup
- **Case**: ABS/Aluminum Enclosure ($10) - ~120mm x 50mm x 30mm, Picatinny mount
- **Total Cost per S1 Unit**: ~$69-$74 (includes bi-color LED)
- **Size**: ~120mm x 50mm x 30mm, ~130mm tall with antenna
- **Weight**: ~200-250g

### Assembly Details

1. **Component Placement**:
   - Mount Nano on a ~50mm x 30mm perfboard.
   - Place LoRa right, HMC5883L left, DS3231 below Nano.
   - Position rangefinder bottom right, aligned with barrel.
   - Secure LiPo bottom left, trigger button top right, power button on side, red/green LED near top (visible).
   - Attach whip antenna to LoRa SMA, angled outward.
2. **Wiring**:
   - Nano 5V → LoRa VCC, GND → GND, D10 (TX) → LoRa RX, D11 (RX) → LoRa TX.
   - Nano A4 (SDA), A5 (SCL) → HMC5883L, DS3231 SDA/SCL.
   - Nano D3 (TX), D4 (RX) → Rangefinder TTL RX/TX.
   - Nano D2 → Trigger Button (GND via pull-up).
   - Nano D8 → Red LED anode (cathode to GND via 220Ω), D9 → Green LED anode (cathode to GND via 220Ω).
   - Nano VIN, GND → LiPo +/– via switch; 5V, GND → USB port.
   - USB port RX/TX (pins 2/12) → Nano RX/TX (shifted to D2/D12 to free D8/D9).
3. **Case Assembly**:
   - Fit into ~120mm x 50mm x 30mm enclosure.
   - Drill holes: SMA (top), USB (side), trigger button (side near top), power button (side near bottom), LED (top), rangefinder lens (front).
   - Attach Picatinny mount, secure PCB, seal lid.
4. **Testing**: Verify power on/off, LoRa TX/RX over 5km NLOS, compass, rangefinder, constant red LED on sensor failure, red flash on failed transmit, green flash on successful transmit, single green flash on startup if sensors ok via serial monitor when powered.

## Base Station (B1)

### Parts List

- **Microcontroller**: Arduino Nano ($5)
- **RF Transceiver**: Ebyte E32-433T20D (LoRa SX1276) ($15)
- **Real-Time Clock**: DS3231 RTC Module ($3)
- **Display**: 0.96” OLED (I2C) ($5)
- **Battery**: 3.7V 2000mAh LiPo ($8) - ~12-15hr runtime (~50mA total)
- **Antenna**: 433 MHz Whip ($3, if not bundled)
- **USB Port**: USB-B or Micro-USB Connector ($1)
- **Rocker Switch**: Up/Down Momentary Switch ($2) - Scrolls mission list
- **Tri-State Switch**: 3-Position Slide Switch ($2) - Selects 81mm mortar, 155mm artillery, 120mm tank
- **Red/Green LED**: Bi-Color 5mm LED with 220Ω Resistors ($2) - Indicates W1 connection status
- **Case**: ABS Enclosure ($7) - ~80mm x 50mm x 35mm
- **Total Cost**: ~$53-$58 (includes red/green LED)
- **Size**: ~80mm x 50mm x 35mm, ~150mm tall with antenna
- **Weight**: ~155-175g (LED adds negligible weight)

### Assembly Details

1. **Component Placement**:
   - Mount Nano on a ~50mm x 40mm perfboard.
   - Place LoRa right, DS3231 below Nano.
   - Position OLED top face, rocker switch below OLED on front, tri-state switch on side, red/green LED near top (visible).
   - Secure LiPo (2000mAh) bottom.
   - Attach whip antenna to LoRa SMA, USB port on side.
2. **Wiring**:
   - Nano 5V → LoRa VCC, GND → GND, D10 (TX) → LoRa RX, D11 (RX) → LoRa TX.
   - Nano A4 (SDA), A5 (SCL) → DS3231, OLED SDA/SCL.
   - Nano D3 → Rocker Up, D4 → Rocker Down (GND via pull-up).
   - Nano D5 → Tri-State Position 1 (81mm), D6 → Position 2 (155mm), D7 → Position 3 (120mm) (GND via pull-up).
   - Nano D8 → Red LED anode (cathode to GND via 220Ω), D9 → Green LED anode (cathode to GND via 220Ω).
   - Nano VIN, GND → LiPo +/– via switch; 5V, GND → USB port.
   - USB port RX/TX (pins 2/12) → Nano RX/TX (shifted to D2/D12 to free D8/D9).
3. **Case Assembly**:
   - Fit into ~80mm x 50mm x 35mm enclosure.
   - Drill holes: SMA (top), USB (side for W1), USB (side for charging), OLED window (top), rocker switch slot (front), tri-state switch slot (side), LED hole (top).
   - Secure PCB, seal lid.
4. **Testing**: Verify LoRa RX/TX over 5km NLOS, OLED display, rocker scrolling, tri-state switch recalculation, red/green LED (green when W1 connected, red when disconnected), and last-known data usage.

## Environmental Sensor (W1)

### Parts List

- **Microcontroller**: Arduino Nano ($5)
- **Compass**: HMC5883L ($3)
- **Wind Speed Sensor**: Modern Device Wind Sensor (Rev. P) ($35 or DIY $15-$20)
- **Environmental Sensor**: BME280 ($5) - Temperature, humidity, pressure
- **USB Cable**: USB-A to Micro-USB ($2) - Power/data from B1 (~1-2m)
- **Red/Green LED**: Bi-Color 5mm LED with 220Ω Resistors ($2) - Red for sensor failure, green flash on startup
- **Case**: ABS Enclosure with Mast Mount ($10) - ~70mm x 40mm x 35mm base, ~100mm mast
- **Total Cost**: ~$62-$67 (upgraded to bi-color LED)
- **Size**: ~70mm x 40mm x 35mm base, ~200mm tall with mast
- **Weight**: ~135-155g

### Assembly Details

1. **Component Placement**:
   - Mount Nano on a ~50mm x 30mm perfboard.
   - Place HMC5883L left, BME280 below Nano, wind sensor right with mast upward, red/green LED near top (visible).
   - Route USB cable from Nano’s Micro-USB out the side.
2. **Wiring**:
   - Nano powered via USB 5V/GND from B1.
   - Nano A4 (SDA), A5 (SCL) → HMC5883L, BME280 SDA/SCL.
   - Nano A0 → Wind sensor analog output.
   - Nano D2 → Red LED anode (cathode to GND via 220Ω), D3 → Green LED anode (cathode to GND via 220Ω).
3. **Case Assembly**:
   - Fit into ~70mm x 40mm x 35mm enclosure.
   - Drill holes: USB (side), wind mast (top), pressure vent (small), LED (top).
   - Secure PCB, seal lid.
4. **Testing**: Connect to B1 via USB, verify power, check sensor outputs via serial monitor, ensure green LED flashes once on startup if sensors ok, red LED constant on sensor failure.

## Conclusion

This RF-Minimized Weapon-Mounted Spotter System for Multi-Weapon Indirect Fire Targeting delivers precise solutions for 81mm mortars, 155mm artillery, or 120mm tanks in Ukraine’s GPS-denied, DF-threatened environment, using TOF ranging and minimal LoRa transmissions (70–140ms per mission) to evade detection over a 5km NLOS range.

# Система прицілювання з мінімальним радіовипромінюванням для непрямого вогню кількома видами зброї

## Вступ

В умовах України без GPS і з загрозою радіочастотного виявлення (DF) станом на 1 березня 2025 року російські перешкоди порушують традиційне позиціонування, а виявлення джерел радіосигналів ставить під загрозу підрозділи, що їх випромінюють. Ця система забезпечує прицілювання на відстані 5 км поза прямою видимістю (NLOS) без використання GPS завдяки вимірюванню часу прольоту (TOF), мінімізуючи радіочастотні передачі для уникнення виявлення DF. Дальність 5 км NLOS відповідає типовому максимальному діапазону 81-мм міномета (наприклад, M252, ~5.6 км зі стандартними снарядами). Спостерігачі ініціюють короткі запити LoRa з перемикачем живлення та червоно-зеленим світлодіодом (постійний червоний при несправності датчиків, червоний спалах при невдалій передачі, зелений спалах при успішному прийомі B1, одиночний зелений спалах при запуску, якщо датчики в нормі), базова станція обчислює вогневі завдання з даних тривалістю ~70-140 мс, перераховуючи рішення для 81-мм міномета, 155-мм артилерії або 120-мм танка на основі трипозиційного перемикача (перерахунок при зміні режиму) та поточних або останніх відомих екологічних даних, з червоно-зеленим світлодіодом для статусу з'єднання W1 та червоно-зеленим світлодіодом для стану датчиків (постійний червоний при несправності, зелений спалах при запуску, якщо все гаразд), усе живиться від батареї 2000 мАг для роботи понад 12 годин у прихованому режимі.

## Огляд системи

- **Пристрої спостереження (S1, S2)**: Кріпляться на зброю, оснащені кнопкою живлення та червоно-зеленим двоколірним світлодіодом (постійний червоний при несправності датчиків, червоний спалах 1 секунда при невдалій передачі вогневих даних, зелений спалах 1 секунда при успішному прийомі B1, одиночний зелений спалах при запуску, якщо датчики в нормі), надсилають запит таймстемпу до B1 при натисканні кнопки увімкнення, обчислюють відстань за TOF, передають дані цілі через LoRa (~70-140 мс RF на завдання) з діапазоном 5 км NLOS.
- **Базова станція (B1)**: Відповідає на запити S1/S2 (~20-40 мс передачі на відстані 5 км NLOS), зберігає список із 10 завдань (11-е перезаписує 1-е, зсуваючи 2-е на 1-е тощо, нове завдання як 10-е), перераховує всі завдання (заряд, азимут, дальність, висота) з поточними даними від W1 або останніми відомими даними, якщо їх немає (перерахунок викликається зміною трипозиційного перемикача), відображає на 0.96-дюймовому OLED з прокруткою за допомогою гойдалки вгору/вниз і трипозиційним перемикачем для вибору зброї (81-мм міномет, 155-мм артилерія, 120-мм танк), має червоно-зелений світлодіод для статусу з'єднання W1 (зелений = підключено, червоний = відключено), живить W1 через USB, працює від батареї 2000 мАг понад 12 годин.
- **Пристрій для екологічних даних і вітру (W1)**: Вимірює швидкість і напрямок вітру, температуру, вологість, тиск, живиться від B1 через USB, передає дані без RF-випромінювання, має червоно-зелений двоколірний світлодіод (постійний червоний при несправності датчиків, одиночний зелений спалах при запуску, якщо датчики в нормі).

## Пристрій спостереження (S1, S2)

### Список компонентів

- **Мікроконтролер**: Arduino Nano ($5)
- **Радіопередавач**: Ebyte E32-433T20D (LoRa SX1276) ($15)
- **Компас**: HMC5883L ($3)
- **Годинник реального часу**: DS3231 RTC Module ($3)
- **Лазерний далекомір**: Wosports H-100 або подібний ($20-$30)
- **Батарея**: 3.7V 1000mAh LiPo ($6)
- **Антена**: 433 MHz Whip ($3, якщо не в комплекті)
- **Кнопка запуску**: Tactile Switch ($1) - Ініціює запит прицілювання
- **Кнопка живлення**: Mini Slide Switch ($1) - Увімкнення/вимкнення
- **Червоно-зелений світлодіод**: Bi-Color 5mm LED з резисторами 220Ω ($2) - Червоний для несправності датчиків або невдалої передачі, зелений для успіху/запуску
- **Корпус**: ABS/Алюмінієвий корпус ($10) - ~120mm x 50mm x 30mm, кріплення Picatinny
- **Загальна вартість одиниці S1**: ~$69-$74 (включає двоколірний світлодіод)
- **Розмір**: ~120mm x 50mm x 30mm, ~130mm з антеною
- **Вага**: ~200-250g

### Деталі складання

1. **Розташування компонентів**:
   - Встановіть Nano на перфоровану плату ~50mm x 30mm.
   - Розташуйте LoRa праворуч, HMC5883L ліворуч, DS3231 під Nano.
   - Розмістіть далекомір у нижньому правому куті, вирівняний зі стволом.
   - Закріпіть LiPo у нижньому лівому куті, кнопку запуску у верхньому правому, кнопку живлення збоку, червоно-зелений світлодіод зверху (видно).
   - Прикріпіть антену до LoRa SMA, спрямовану назовні.
2. **Проводка**:
   - Nano 5V → LoRa VCC, GND → GND, D10 (TX) → LoRa RX, D11 (RX) → LoRa TX.
   - Nano A4 (SDA), A5 (SCL) → HMC5883L, DS3231 SDA/SCL.
   - Nano D3 (TX), D4 (RX) → Rangefinder TTL RX/TX.
   - Nano D2 → Trigger Button (GND через підтягувач).
   - Nano D8 → Анод червоного світлодіода (катод до GND через 220Ω), D9 → Анод зеленого світлодіода (катод до GND через 220Ω).
   - LiPo + → Кнопка живлення (один бік), інший бік → Nano VIN; LiPo – → GND.
   - USB порт RX/TX (піни 2/12) → Nano RX/TX (зміщені на D2/D12 для звільнення D8/D9).
3. **Складання корпусу**:
   - Помістіть у корпус ~120mm x 50mm x 30mm.
   - Просвердліть отвори: SMA (зверху), USB (збоку), кнопка запуску (збоку зверху), кнопка живлення (збоку знизу), світлодіод (зверху), лінза далекоміра (спереду).
   - Прикріпіть кріплення Picatinny, закріпіть плату, закрийте кришку.
4. **Тестування**: Перевірте увімкнення/вимкнення, передачу/прийом LoRa на 5 км NLOS, компас, далекомір, постійний червоний світлодіод при несправності датчиків, червоний спалах при невдалій передачі, зелений спалах при успішній передачі, одиночний зелений спалах при запуску, якщо датчики в нормі, через монітор послідовного порту.

## Базова станція (B1)

### Список компонентів

- **Мікроконтролер**: Arduino Nano ($5)
- **Радіопередавач**: Ebyte E32-433T20D (LoRa SX1276) ($15)
- **Годинник реального часу**: DS3231 RTC Module ($3)
- **Дисплей**: 0.96” OLED (I2C) ($5)
- **Батарея**: 3.7V 2000mAh LiPo ($8) - ~12-15 годин роботи (~50mA загалом)
- **Антена**: 433 MHz Whip ($3, якщо не в комплекті)
- **USB порт**: USB-B або Micro-USB роз’єм ($1)
- **Гойдалка**: Up/Down Momentary Switch ($2) - Прокрутка списку завдань
- **Трьохпозиційний перемикач**: 3-Position Slide Switch ($2) - Вибір 81-мм міномета, 155-мм артилерії, 120-мм танка
- **Червоно-зелений світлодіод**: Bi-Color 5mm LED з резисторами 220Ω ($2) - Показує статус з'єднання W1
- **Корпус**: ABS корпус ($7) - ~80mm x 50mm x 35mm
- **Загальна вартість**: ~$53-$58 (включає двоколірний світлодіод)
- **Розмір**: ~80mm x 50mm x 35mm, ~150mm з антеною
- **Вага**: ~155-175g (світлодіод додає незначну вагу)

### Деталі складання

1. **Розташування компонентів**:
   - Встановіть Nano на перфоровану плату ~50mm x 40mm.
   - Розташуйте LoRa праворуч, DS3231 під Nano.
   - Розмістіть OLED зверху лицьовою стороною, гойдалку під OLED спереду, трипозиційний перемикач збоку, червоно-зелений світлодіод зверху (видно).
   - Закріпіть LiPo (2000mAh) знизу.
   - Прикріпіть антену до LoRa SMA, USB порт збоку.
2. **Проводка**:
   - Nano 5V → LoRa VCC, GND → GND, D10 (TX) → LoRa RX, D11 (RX) → LoRa TX.
   - Nano A4 (SDA), A5 (SCL) → DS3231, OLED SDA/SCL.
   - Nano D3 → Гойдалка вгору, D4 → Гойдалка вниз (GND через підтягувач).
   - Nano D5 → Позиція 1 трипозиційного перемикача (81mm), D6 → Позиція 2 (155mm), D7 → Позиція 3 (120mm) (GND через підтягувач).
   - Nano D8 → Анод червоного світлодіода (катод до GND через 220Ω), D9 → Анод зеленого світлодіода (катод до GND через 220Ω).
   - Nano VIN, GND → LiPo +/– через перемикач; 5V, GND → USB порт.
   - USB порт RX/TX (піни 2/12) → Nano RX/TX (зміщені на D2/D12 для звільнення D8/D9).
3. **Складання корпусу**:
   - Помістіть у корпус ~80mm x 50mm x 35mm.
   - Просвердліть отвори: SMA (зверху), USB (збоку для W1), USB (збоку для зарядки), вікно OLED (зверху), слот для гойдалки (спереду), слот для трипозиційного перемикача (збоку), отвір для світлодіода (зверху).
   - Закріпіть плату, закрийте кришку.
4. **Тестування**: Перевірте передачу/прийом LoRa на 5 км NLOS, дисплей OLED, прокрутку гойдалкою, перерахунок трипозиційним перемикачем, червоно-зелений світлодіод (зелений при підключенні W1, червоний при відключенні), використання останніх відомих даних.

## Пристрій для екологічних даних і вітру (W1)

### Список компонентів

- **Мікроконтролер**: Arduino Nano ($5)
- **Компас**: HMC5883L ($3)
- **Датчик швидкості вітру**: Modern Device Wind Sensor (Rev. P) ($35 або DIY $15-$20)
- **Екологічний датчик**: BME280 ($5) - Температура, вологість, тиск
- **USB кабель**: USB-A до Micro-USB ($2) - Живлення/дані від B1 (~1-2м)
- **Червоно-зелений світлодіод**: Bi-Color 5mm LED з резисторами 220Ω ($2) - Червоний для несправності датчиків, зелений спалах при запуску
- **Корпус**: ABS корпус з кріпленням для щогли ($10) - ~70mm x 40mm x 35mm база, ~100mm щогла
- **Загальна вартість**: ~$62-$67 (оновлено до двоколірного світлодіода)
- **Розмір**: ~70mm x 40mm x 35mm база, ~200mm з щоглою
- **Вага**: ~135-155g

### Деталі складання

1. **Розташування компонентів**:
   - Встановіть Nano на перфоровану плату ~50mm x 30mm.
   - Розташуйте HMC5883L ліворуч, BME280 під Nano, датчик вітру праворуч із щоглою вгору, червоно-зелений світлодіод зверху (видно).
   - Прокладіть USB кабель від Micro-USB Nano збоку.
2. **Проводка**:
   - Живлення Nano через USB 5V/GND від B1.
   - Nano A4 (SDA), A5 (SCL) → HMC5883L, BME280 SDA/SCL.
   - Nano A0 → Аналоговий вихід датчика вітру.
   - Nano D2 → Анод червоного світлодіода (катод до GND через 220Ω), D3 → Анод зеленого світлодіода (катод до GND через 220Ω).
3. **Складання корпусу**:
   - Помістіть у корпус ~70mm x 40mm x 35mm.
   - Просвердліть отвори: USB (збоку), щогла вітру (зверху), вентиляційний отвір для тиску (маленький), світлодіод (зверху).
   - Закріпіть плату, закрийте кришку.
4. **Тестування**: Підключіть до B1 через USB, перевірте живлення, вивід даних датчиків через монітор послідовного порту, переконайтеся, що зелений світлодіод спалахує раз при запуску, якщо датчики в нормі, червоний світлодіод постійний при несправності.

## Висновок

Ця система прицілювання з мінімальним радіовипромінюванням для непрямого вогню кількома видами зброї забезпечує точні рішення для 81-мм мінометів, 155-мм артилерії або 120-мм танків у GPS-відмовному, DF-загрозливому середовищі України, використовуючи TOF вимірювання дальності та мінімальні передачі LoRa (~70-140 мс на завдання) для уникнення виявлення на відстані 5 км NLOS (~5.6 км для 81-мм м