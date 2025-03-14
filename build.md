# Arduino Nano Modules

| **Category**         | **Part Description**                                                                                   | **Quantity** | **Connections (Nano Pin → Module Pin)**                                                                                                   |
|----------------------|--------------------------------------------------------------------------------------------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| **Nano**             | DUBEUYEW 3pcs Nano Soldered Module, Nano Board CH340 Chip, 5V 16MHz for Arduino with Cable and Adapter | 3            | N/A (Base Nano board)                                                                                                                    |
| **Modules - Coms**   | Acfthepiey -NEO6MV2 New NEO-6M GPS Module NEO6MV2 with Flight Control EEPROM APM2.5 Antenna           | 1            | Nano D2 → GPS TX, Nano D3 → GPS RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D2/D3)                                              |
| **Modules - Coms**   | 2pc LORA Wireless Module 22dBm 433~475MHz 8KM LR01-SMD Low Power Wireless Transceiver UART             | 2            | Nano D10 → LORA TX, Nano D11 → LORA RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D10/D11)                                        |
| **Modules - Sensors**| DAOKAI Three-axis magnetic sensor GY-273 QMC5883L HMC5883L Triple Axis Compass Magnetometer (Pack of 3) | 3            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Sensors**| Diymore 2PCS GY-BME280 High Precision Digital Sensor Breakout Barometric Pressure Temperature Humidity | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VIN (I2C shared bus)                                                             |
| **Modules - Sensors**| Qyrugcxs Tf-Luna Lidar Range Sensor Module 8M Range Low Power Tof Range Principle                     | 1            | Nano D4 → Lidar TX, Nano D5 → Lidar RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D4/D5)                                          |
| **Modules - Sensors**| DollaTek 5Pcs Tiny RTC I2C DS1307 AT24C32 Real Time Clock Module For Arduino AVR PIC 51 ARM            | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Sensors**| KY-022 Set IR Receiver Infrared Receiver CHQ1838 Sensor Module                                        | 5            | Nano D12 → S (Signal), Nano GND → GND, Nano 5V → VCC (Digital input on D12)                                                              |
| **Modules - UI**     | HALJIA 5Pcs Five Direction Navigation Button Module DIY Electronic PCB Board                          | 5            | Each connected to a separate PCF8574 expander: com → GND, up → P0, down → P1, left → P2, right → P3, mid → P4, set → P5, reset → Nano RESET; PCF8574 expanders connected to Nano via I2C (A4 → SDA, A5 → SCL), with unique addresses set via jumpers |
| **Modules - UI**     | Universal 4 Key Push Button Switch Module 4 Channel Keyboard Board Compatible by Garosa                | 1            | Nano A0 → Key1, Nano A1 → Key2, Nano A2 → Key3, Nano A3 → Key4, Nano GND → GND, Nano 5V → VCC                                           |
| **Modules - UI**     | Fasizi 2Pcs 0.96" I2C IIC SPI Serial 128x64 OLED Display Module Board with Pin Headers                | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Pack of 10)                         | 10           | Nano D6 → 220Ω Resistor → Red Anode, Nano D7 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Additional)                         | 1            | Nano D8 → 220Ω Resistor → Red Anode, Nano D9 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | UMTMedia® 30pcs 220 ohm O - 1/4W Watt Metal Film Resistors 0.25 ±1%                                   | 30           | N/A (Used in series with LED anodes as above)                                                                                            |
| **Modules**          | Youmile 5 pcs PCF8574 IO Expansion Board PCF8574 I/O Expander I2C Evaluation Develop Module with DuPont Cable for Arduino & Raspberry Pi | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus, each with unique address set via jumpers)                   |
| **Modules**          | AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module (Pack of 3)          | 3            | Shield 5V Out → Nano 5V, Shield 3.3V Out → Nano 3.3V, Shield GND → Nano GND (Power output to Nano)                                        |
# Arduino Nano to Module Connections

| Nano Pin (Left) | Connected To (Left)                                          | Connected To (Right)                                         | Nano Pin (Right) |
|-----------------|--------------------------------------------------------------|--------------------------------------------------------------|------------------|
| D13             | Free                                                         | KY-022 IR Receiver S (Signal)                                | D12              |
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
| 5V              | Multiple modules' VCC (GPS, LORA, Lidar, OLED, PCF8574, KY-022, etc.) | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, KY-022, etc.) | GND              |
| RST             | Navigation buttons' reset pins                               | (Same as left)                                               | RST              |
| GND             | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, KY-022, etc.) | Free                                                         | RX (D0)          |
| VIN             | Battery shield VIN                                           | Free                                                         | TX (D1)          |  |

# Module Connections to Arduino Nano

Below are the connections for each module to the Arduino Nano, organized by module. Each table lists the module's pins and the corresponding Arduino Nano pins they connect to.

## GPS Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D2       |
| RX         | D3       |

**Note:** Uses SoftwareSerial on pins D2 (RX) and D3 (TX).

## LORA Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D10      |
| RX         | D11      |

**Note:** Uses SoftwareSerial on pins D10 (RX) and D11 (TX).

## Lidar Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D4       |
| RX         | D5       |

**Note:** Uses SoftwareSerial on pins D4 (RX) and D5 (TX).

## OLED Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

## Magnetic Sensor

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

## BME280

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

## RTC Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

## PCF8574 (x5)

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** There are 5 PCF8574 modules, each configured with a unique I2C address set via jumpers. They share the I2C bus on Nano pins A4 (SDA) and A5 (SCL).

## HALJIA 5Pcs Five Direction Navigation Button Module (x5, each via separate PCF8574)

| Module Pin | Connection                 |
|------------|----------------------------|
| com        | GND                        |
| up         | PCF8574 P0 (I2C on A4, A5) |
| down       | PCF8574 P1 (I2C on A4, A5) |
| left       | PCF8574 P2 (I2C on A4, A5) |
| right      | PCF8574 P3 (I2C on A4, A5) |
| mid        | PCF8574 P4 (I2C on A4, A5) |
| set        | PCF8574 P5 (I2C on A4, A5) |
| reset      | Nano RST                   |

**Note:** Each of the 5 navigation modules is connected to its own PCF8574 expander, with unique I2C addresses set via jumpers. The PCF8574 expanders are connected to the Arduino Nano via the I2C bus (A4 → SDA, A5 → SCL).

## Universal 4 Key Push Button Switch Module

| Module Pin | Nano Pin |
|------------|----------|
| GND        | GND      |
| K1         | A0       |
| K2         | A1       |
| K3         | A2       |
| K4         | A3       |

**Note:** The key pins (K1 to K4) are connected to the Arduino Nano's analog pins A0 to A3, which are used as digital inputs with internal pull-up resistors.

## AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module

| Module Pin | Nano Pin |
|------------|----------|
| 5V Out     | 5V       |
| 3V Out     | 3.3V     |
| GND        | GND      |
| Micro USB  | VIN      |

**Note:** This module provides power to the Nano. The **5V Out** connects to the Nano’s 5V pin to supply power, **3V Out** connects to the 3.3V pin if needed, and **GND** connects to any Nano GND pin. The **Micro USB** port accepts external 5V input and connects to VIN for powering the Nano when not using the battery.

## KY-022 Set IR Receiver Infrared Receiver CHQ1838 Sensor Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| S (Signal) | D12      |

**Note:** The KY-022 IR Receiver module detects infrared signals (typically at 38kHz) and outputs a digital signal on the **S (Signal)** pin, connected to Nano pin D12. This pin can be read using an Arduino library like IRremote to decode IR remote control signals.

**General Note:** Multiple modules (OLED, PCF8574, Magnetic Sensor, BME280, RTC) share the I2C bus on Nano pins A4 (SDA) and A5 (SCL). Ensure each I2C device has a unique address to avoid communication conflicts. The KY-022 uses a digital pin (D12) and does not share the I2C bus.

---

### Explanation of KY-022 Addition
- **Pinout**: The KY-022 module typically has three pins: VCC (power), GND (ground), and S (signal output). It’s connected to:
  - **VCC** → Nano 5V for power.
  - **GND** → Nano GND for ground.
  - **S (Signal)** → Nano D12, a free digital pin, for reading IR signals. D12 was chosen as it’s unused in the current setup (D0-D11 are occupied, D13 is free but often reserved for the onboard LED).
- **Functionality**: The KY-022 detects IR signals (e.g., from a remote control) and outputs a digital signal that can be processed by the Nano using libraries like IRremote.
- **Integration**: Unlike the I2C modules, the KY-022 uses a digital input pin, keeping it separate from the crowded I2C bus on A4/A5.

This updated Markdown content includes all modules with their pin connections, now featuring the KY-022 IR Receiver module, and is ready to be copied into a `.md` file for your project documentation.

### Explanation of AZDelivery 18650 Module Pinout
The **AZDelivery 18650 Lithium Li-ion Battery Expansion Shield** is a power supply module with the following key features:
- **5V Out**: Outputs 5V from the 18650 battery (via a boost converter), connected to the Nano’s 5V pin to power the board and modules.
- **3V Out**: Outputs 3V (or 3.3V depending on the model), connected to the Nano’s 3.3V pin if additional 3.3V power is needed (optional, as the Nano has its own 3.3V regulator).
- **GND**: Ground connection, linked to any Nano GND pin.
- **Micro USB**: Input for charging the 18650 battery or powering the Nano directly when no battery is present; connects to VIN to supply power to the Nano.

This module does not require data pins (like I2C or serial) as it’s a power source, not a communication device. The pinout reflects its role in supplying power to the Nano, integrating seamlessly with the existing setup. You can copy this entire Markdown block into your `.md` file for documentation.

**Note:** The key pins (K1 to K4) are connected to the Arduino Nano's analog pins A0 to A3, which are used as digital inputs with internal pull-up resistors.

**General Note:** Multiple modules (OLED, PCF8574, Magnetic Sensor, BME280, RTC) share the I2C bus on Nano pins A4 (SDA) and A5 (SCL). Ensure each I2C device has a unique address to avoid communication conflicts.