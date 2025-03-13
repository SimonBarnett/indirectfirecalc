| **Category**         | **Part Description**                                                                                   | **Quantity** | **Connections (Nano Pin → Module Pin)**                                                                                                   |
|----------------------|--------------------------------------------------------------------------------------------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| **Nano**             | DUBEUYEW 3pcs Nano Soldered Module, Nano Board CH340 Chip, 5V 16MHz for Arduino with Cable and Adapter | 3            | N/A (Base Nano board)                                                                                                                    |
| **Modules - Coms**   | Acfthepiey -NEO6MV2 New NEO-6M GPS Module NEO6MV2 with Flight Control EEPROM APM2.5 Antenna           | 1            | Nano D2 → GPS TX, Nano D3 → GPS RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D2/D3)                                              |
| **Modules - Coms**   | 2pc LORA Wireless Module 22dBm 433~475MHz 8KM LR01-SMD Low Power Wireless Transceiver UART             | 2            | Nano D10 → LORA TX, Nano D11 → LORA RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D10/D11)                                        |
| **Modules - Sensors**| DAOKAI Three-axis magnetic sensor GY-273 QMC5883L HMC5883L Triple Axis Compass Magnetometer (Pack of 3) | 3            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Sensors**| Diymore 2PCS GY-BME280 High Precision Digital Sensor Breakout Barometric Pressure Temperature Humidity | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VIN (I2C shared bus)                                                             |
| **Modules - Sensors**| Qyrugcxs Tf-Luna Lidar Range Sensor Module 8M Range Low Power Tof Range Principle                     | 1            | Nano D4 → Lidar TX, Nano D5 → Lidar RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D4/D5)                                          |
| **Modules - Sensors**| DollaTek 5Pcs Tiny RTC I2C DS1307 AT24C32 Real Time Clock Module For Arduino AVR PIC 51 ARM            | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - UI**     | HALJIA 5Pcs Five Direction Navigation Button Module DIY Electronic PCB Board                          | 5            | Each connected to a separate PCF8574 expander: com → GND, up → P0, down → P1, left → P2, right → P3, mid → P4, set → P5, reset → Nano RESET; PCF8574 expanders connected to Nano via I2C (A4 → SDA, A5 → SCL), with unique addresses set via jumpers |
| **Modules - UI**     | Universal 4 Key Push Button Switch Module 4 Channel Keyboard Board Compatible by Garosa                | 1            | Nano A0 → Key1, Nano A1 → Key2, Nano A2 → Key3, Nano A3 → Key4, Nano GND → GND, Nano 5V → VCC                                           |
| **Modules - UI**     | Fasizi 2Pcs 0.96" I2C IIC SPI Serial 128x64 OLED Display Module Board with Pin Headers                | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Pack of 10)                         | 10           | Nano D6 → 220Ω Resistor → Red Anode, Nano D7 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Additional)                         | 1            | Nano D8 → 220Ω Resistor → Red Anode, Nano D9 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | UMTMedia® 30pcs 220 ohm O - 1/4W Watt Metal Film Resistors 0.25 ±1%                                   | 30           | N/A (Used in series with LED anodes as above)                                                                                            |
| **Modules**          | Youmile 5 pcs PCF8574 IO Expansion Board PCF8574 I/O Expander I2C Evaluation Develop Module with DuPont Cable for Arduino & Raspberry Pi | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus, each with unique address set via jumpers)                   |
| **Modules**          | AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module (Pack of 3)          | 3            | Shield 5V Out → Nano 5V, Shield 3.3V Out → Nano 3.3V, Shield GND → Nano GND (Power output to Nano)                                        |

| Nano Pin (Left) | Connected To (Left)                                          | Connected To (Right)                                         | Nano Pin (Right) |
|-----------------|--------------------------------------------------------------|--------------------------------------------------------------|------------------|
| D13             | Free                                                         | Free                                                         | D12              |
| 3.3V            | Battery shield 3.3V out                                      | LORA module RX pin                                           | D11              |
| REF             | Free                                                         | LORA module TX pin                                           | D10              |
| A0              | Push button 1                                                | LED green anode (via 220Ω resistor)                          | D9               |
| A1              | Push button 2                                                | LED red anode (via 220Ω resistor)                            | D8               |
| A2              | Push button 3                                                | LED green anode (via 220Ω resistor)                          | D7               |
| A3              | Push button 4                                                | LED red anode (via 220Ω resistor)                            | D6               |
| A4              | I2C SDA: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module RX pin                                          | D5               |
| A5              | I2C SCL: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module TX pin                                          | D4               |
| A6              | Free                                                         | GPS module RX pin                                            | D3               |
| A7              | Free                                                         | GPS module TX pin                                            | D2               |
| 5V              | Multiple modules' VCC (GPS, LORA, Lidar, OLED, PCF8574, etc.)| Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, etc.)| GND              |
| RST             | Navigation buttons' reset pins                               | (Same as left)                                               | RST              |
| GND             | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, etc.)| Free                                                         | RX (D0)          |
| VIN             | Battery shield VIN                                           | Free                                                         | TX (D1)          |

| Module Name      | Module Pin | Nano Pin |
|------------------|------------|----------|
| GPS Module       | VCC        | 5V       |
| GPS Module       | GND        | GND      |
| GPS Module       | TX         | D2       |
| GPS Module       | RX         | D3       |
| LORA Module      | VCC        | 5V       |
| LORA Module      | GND        | GND      |
| LORA Module      | TX         | D10      |
| LORA Module      | RX         | D11      |
| Lidar Module     | VCC        | 5V       |
| Lidar Module     | GND        | GND      |
| Lidar Module     | TX         | D4       |
| Lidar Module     | RX         | D5       |
| OLED Module      | VCC        | 5V       |
| OLED Module      | GND        | GND      |
| OLED Module      | SDA        | A4       |
| OLED Module      | SCL        | A5       |
| PCF8574          | VCC        | 5V       |
| PCF8574          | GND        | GND      |
| PCF8574          | SDA        | A4       |
| PCF8574          | SCL        | A5       |
| Magnetic Sensor  | VCC        | 5V       |
| Magnetic Sensor  | GND        | GND      |
| Magnetic Sensor  | SDA        | A4       |
| Magnetic Sensor  | SCL        | A5       |
| BME280           | VCC        | 5V       |
| BME280           | GND        | GND      |
| BME280           | SDA        | A4       |
| BME280           | SCL        | A5       |
| RTC Module       | VCC        | 5V       |
| RTC Module       | GND        | GND      |
| RTC Module       | SDA        | A4       |
| RTC Module       | SCL        | A5       |