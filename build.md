# Arduino Nano Modules

| **Category**         | **Part Description**                                                                                   | **Quantity** | **Connections (Nano Pin → Module Pin)**                                                                                                   |
|----------------------|--------------------------------------------------------------------------------------------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| **Nano**             | DUBEUYEW 3pcs Nano Soldered Module, Nano Board CH340 Chip, 5V 16MHz for Arduino with Cable and Adapter | 3            | N/A (Base Nano board)                                                                                                                    |
| **Modules - Coms**   | Acfthepiey -NEO6MV2 New NEO-6M GPS Module NEO6MV2 with Flight Control EEPROM APM2.5 Antenna           | 1            | Nano D2 → GPS TX, Nano D3 → GPS RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D2/D3)                                              |
| **Modules - Coms**   | 2pc LORA Wireless Module 22dBm 433~475MHz 8KM LR01-SMD Low Power Wireless Transceiver UART             | 2            | Nano D10 → LORA TX, Nano D11 → LORA RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D10/D11)                                        |
| **Modules - Coms**   | ALAMSCN Digital 38KHz Infrared IR Receiver Sensor Module with Transmitter Module Kit Set              | 5            | Receiver: Nano A2 → DAT, Nano GND → GND, Nano 5V → VCC; Transmitter: Nano A3 → DAT, Nano GND → GND, Nano 5V → VCC                    |
| **Modules - Sensors**| DAOKAI Three-axis magnetic sensor GY-273 QMC5883L HMC5883L Triple Axis Compass Magnetometer (Pack of 3) | 3            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Sensors**| Diymore 2PCS GY-BME280 High Precision Digital Sensor Breakout Barometric Pressure Temperature Humidity | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Sensors**| Qyrugcxs Tf-Luna Lidar Range Sensor Module 8M Range Low Power Tof Range Principle                     | 1            | Nano D4 → Lidar TX, Nano D5 → Lidar RX, Nano GND → GND, Nano 5V → VCC (SoftwareSerial on D4/D5)                                          |
| **Modules - Sensors**| DollaTek 5Pcs Tiny RTC I2C DS1307 AT24C32 Real Time Clock Module For Arduino AVR PIC 51 ARM            | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - UI**     | HALJIA 5Pcs Five Direction Navigation Button Module DIY Electronic PCB Board                          | 5            | Connected to a PCF8574 expander: com → GND, up → P0, down → P1, left → P2, right → P3, mid → P4, set → P5; PCF8574 connected to Nano via I2C (A4 → SDA, A5 → SCL) |
| **Modules - UI**     | Universal 4 Key Push Button Switch Module 4 Channel Keyboard Board Compatible by Garosa                | 1            | Nano D6 → Key1, Nano D7 → Key2, Nano D8 → Key3, Nano D9 → Key4, Nano GND → GND, Nano 5V → VCC (Direct connection to Nano pins)       |
| **Modules - UI**     | Youmile 5 pcs PCF8574 IO Expansion Board PCF8574 I/O Expander I2C Evaluation Develop Module with DuPont Cable for Arduino & Raspberry Pi | 4            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - UI**     | Fasizi 2Pcs 0.96" I2C IIC SPI Serial 128x64 OLED Display Module Board with Pin Headers                | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - UI**     | DAOKAI 100PCS 2Pin Tactile Tact Push Button 6x6x5mm Vertical Momentary Switch Miniature Button Electronic Components for Panel PCB | 1            | Nano RST → Switch Pin 1, Nano GND → Switch Pin 2 (Hardware reset switch with 10kΩ pull-up to 5V)                                          |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Pack of 10)                         | 10           | N/A (No longer connected to Nano pins)                                                                                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Additional)                         | 1            | N/A (No longer connected to Nano pins)                                                                                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Linked to Nano Light)               | 1            | N/A (No longer connected to Nano pins)                                                                                                   |
| **Modules - Bulb**   | UMTMedia® 30pcs 220 ohm O - 1/4W Watt Metal Film Resistors 0.25 ±1%                                   | 30           | N/A (Used as needed in the circuit)                                                                                                      |
| **Modules - Power**  | AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module (Pack of 3)          | 3            | Shield 5V Out → Nano 5V, Shield 3.3V Out → Nano 3.3V, Shield GND → Nano GND (Power output to Nano)                                        |

# Arduino Nano to Module Connections

| Nano Pin (Left) | Connected To (Left)                                          | Connected To (Right)                                         | Nano Pin (Right) |
|-----------------|--------------------------------------------------------------|--------------------------------------------------------------|------------------|
| D13             | Bipolar LED 2 - pin2                                         | Bipolar LED 2 - pin1 (via 220Ω resistor)                     | D12              |
| 3.3V            | Battery shield 3.3V out                                      | LORA module RX pin                                           | D11              |
| REF             | Free                                                         | LORA module TX pin                                           | D10              |
| A0              | Bipolar LED 1 - pin1 (via 220Ω resistor)                     | Push Button 4 (Key4)                                         | D9               |
| A1              | Bipolar LED 1 - pin2                                         | Push Button 3 (Key3)                                         | D8               |
| A2              | ALAMSCN IR Receiver DAT                                      | Push Button 2 (Key2)                                         | D7               |
| A3              | ALAMSCN IR Transmitter DAT                                   | Push Button 1 (Key1)                                         | D6               |
| A4              | I2C SDA: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module RX pin                                          | D5               |
| A5              | I2C SCL: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module TX pin                                          | D4               |
| A6              | Free                                                         | GPS module RX pin                                            | D3               |
| A7              | Free                                                         | GPS module TX pin                                            | D2               |
| 5V              | Multiple modules' VCC (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) | GND              |
| RST             | DAOKAI Tactile Push Button Switch Pin 1 (with 10kΩ to 5V)    | (Same as left)                                               | RST              |
| GND             | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) + DAOKAI Switch Pin 2 | Free                                                         | RX (D0)          |
| VIN             | Battery shield VIN                                           | Free                                                         | TX (D1)          |

# Module Connections to Arduino Nano

Below are the detailed connections for each module to the Arduino Nano, organized by module and ordered as they appear in the Arduino Nano Modules table. Each table maps only one instance of each module type.

### Modules - Coms

#### GPS Module
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D2       |
| RX         | D3       |

**Note:** Uses SoftwareSerial on pins D2 (RX) and D3 (TX).

#### LORA Module
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D10      |
| RX         | D11      |

**Note:** Uses SoftwareSerial on pins D10 (RX) and D11 (TX).

#### ALAMSCN Digital 38KHz Infrared IR Receiver Sensor Module with Transmitter Module Kit Set
| Module Pin       | Nano Pin |
|------------------|----------|
| Receiver VCC     | 5V       |
| Receiver GND     | GND      |
| Receiver DAT     | A2       |
| Transmitter VCC  | 5V       |
| Transmitter GND  | GND      |
| Transmitter DAT  | A3       |

**Note:** One pair mapped here. Receiver DAT to A2 (input), Transmitter DAT to A3 (output).

### Modules - Sensors

#### Magnetic Sensor
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to I2C bus on A4 (SDA) and A5 (SCL).

#### BME280
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to I2C bus on A4 (SDA) and A5 (SCL).

#### Lidar Module
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D4       |
| RX         | D5       |

**Note:** Uses SoftwareSerial on D4 (RX) and D5 (TX).

#### RTC Module
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to I2C bus on A4 (SDA) and A5 (SCL).

### Modules - UI

#### HALJIA Five Direction Navigation Button Module
| Module Pin | Connection                 |
|------------|----------------------------|
| com        | GND                        |
| up         | PCF8574 P0 (I2C on A4, A5) |
| down       | PCF8574 P1 (I2C on A4, A5) |
| left       | PCF8574 P2 (I2C on A4, A5) |
| right      | PCF8574 P3 (I2C on A4, A5) |
| mid        | PCF8574 P4 (I2C on A4, A5) |
| set        | PCF8574 P5 (I2C on A4, A5) |

**Note:** One module mapped, connected to a PCF8574 via I2C (A4 → SDA, A5 → SCL). Reset pin (previously on RST) is now software-managed via P5.

#### Universal 4 Key Push Button Switch Module
| Module Pin | Nano Pin |
|------------|----------|
| GND        | GND      |
| VCC        | 5V       |
| Key1       | D6       |
| Key2       | D7       |
| Key3       | D8       |
| Key4       | D9       |

**Note:** Directly connected to Nano pins D6–D9.

#### PCF8574
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** One PCF8574 mapped, connected to I2C bus on A4 (SDA) and A5 (SCL). Used for navigation buttons.

#### OLED Module
| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to I2C bus on A4 (SDA) and A5 (SCL).

#### DAOKAI Tactile Push Button Switch
| Module Pin | Nano Pin |
|------------|----------|
| Pin 1      | RST      |
| Pin 2      | GND      |

**Note:** 2-pin momentary switch with a 10kΩ pull-up resistor from RST to 5V. Pressing pulls RST low for a hardware reset.

### Modules - Bulb
- **Note:** All Red/Green Bi-Colour LEDs are no longer connected to Nano pins.

### Modules - Power

#### AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module
| Module Pin | Nano Pin |
|------------|----------|
| 5V Out     | 5V       |
| 3V Out     | 3.3V     |
| GND        | GND      |
| Micro USB  | VIN      |

**Note:** Powers the Nano. 5V Out to Nano 5V, 3V Out to 3.3V if needed, GND to Nano GND, Micro USB to VIN for external 5V input.

---

### General Notes
- **I2C Bus:** Shared by OLED, PCF8574, Magnetic Sensor, BME280, and RTC on A4 (SDA) and A5 (SCL). Ensure unique I2C addresses for each PCF8574 (set via A0–A2 pins).
- **Reset Changes:** 
  - HALJIA Navigation Button’s reset is disconnected from RST and now software-managed via PCF8574 P5.
  - New DAOKAI Tactile Switch on RST provides hardware reset.
- **Pin Availability:**
  - **D0, D1:** Free for digital I/O.
  - **A6, A7:** Free for analog inputs (or digital inputs, not outputs).
  - **D12, D13, A0, A1:** Used by LEDs but available if LEDs are removed.
- **LEDs Removed:** Previously on D6–D9 and D13, now disconnected.

This updated build reflects the addition of the DAOKAI Tactile Switch as a hardware reset on RST, with the navigation button’s reset repurposed for software control. Let me know if you need further adjustments!