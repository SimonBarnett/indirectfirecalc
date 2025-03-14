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
| **Modules - UI**     | HALJIA 5Pcs Five Direction Navigation Button Module DIY Electronic PCB Board                          | 5            | Connected to a PCF8574 expander: com → GND, up → P0, down → P1, left → P2, right → P3, mid → P4, set → P5, reset → Nano RST; PCF8574 connected to Nano via I2C (A4 → SDA, A5 → SCL) |
| **Modules - UI**     | Universal 4 Key Push Button Switch Module 4 Channel Keyboard Board Compatible by Garosa                | 1            | Connected to a PCF8574 expander: Key1 → P0, Key2 → P1, Key3 → P2, Key4 → P3, GND → GND, VCC → 5V; PCF8574 connected to Nano via I2C (A4 → SDA, A5 → SCL) |
| **Modules - UI**     | Youmile 5 pcs PCF8574 IO Expansion Board PCF8574 I/O Expander I2C Evaluation Develop Module with DuPont Cable for Arduino & Raspberry Pi | 5            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - UI**     | Fasizi 2Pcs 0.96" I2C IIC SPI Serial 128x64 OLED Display Module Board with Pin Headers                | 2            | Nano A4 → SDA, Nano A5 → SCL, Nano GND → GND, Nano 5V → VCC (I2C shared bus)                                                             |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Pack of 10)                         | 10           | Nano D6 → 220Ω Resistor → Red Anode, Nano D7 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Additional)                         | 1            | Nano D8 → 220Ω Resistor → Red Anode, Nano D9 → 220Ω Resistor → Green Anode, Nano GND → Common Cathode                                   |
| **Modules - Bulb**   | Red/Green Bi-Colour 5mm Diffused LED 60° 2V 25mA Light Lamp Bulb (Linked to Nano Light)               | 1            | Nano D13 → 220Ω Resistor → Red Anode, Nano GND → Common Cathode                                                                          |
| **Modules - Bulb**   | UMTMedia® 30pcs 220 ohm O - 1/4W Watt Metal Film Resistors 0.25 ±1%                                   | 30           | N/A (Used in series with LED anodes as above)                                                                                            |
| **Modules - Power**  | AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module (Pack of 3)          | 3            | Shield 5V Out → Nano 5V, Shield 3.3V Out → Nano 3.3V, Shield GND → Nano GND (Power output to Nano)                                        |

# Arduino Nano to Module Connections

| Nano Pin (Left) | Connected To (Left)                                          | Connected To (Right)                                         | Nano Pin (Right) |
|-----------------|--------------------------------------------------------------|--------------------------------------------------------------|------------------|
| D13             | LED Red Anode (via 220Ω resistor)                            | Free                                                         | D12              |
| 3.3V            | Battery shield 3.3V out                                      | LORA module RX pin                                           | D11              |
| REF             | Free                                                         | LORA module TX pin                                           | D10              |
| A0              | Free                                                         | LED green anode (via 220Ω resistor)                          | D9               |
| A1              | Free                                                         | LED red anode (via 220Ω resistor)                            | D8               |
| A2              | ALAMSCN IR Receiver DAT                                      | LED green anode (via 220Ω resistor)                          | D7               |
| A3              | ALAMSCN IR Transmitter DAT                                   | LED red anode (via 220Ω resistor)                            | D6               |
| A4              | I2C SDA: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module RX pin                                          | D5               |
| A5              | I2C SCL: Magnetic Sensor, BME280, RTC, OLED, PCF8574         | Lidar module TX pin                                          | D4               |
| A6              | Free                                                         | GPS module RX pin                                            | D3               |
| A7              | Free                                                         | GPS module TX pin                                            | D2               |
| 5V              | Multiple modules' VCC (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) | GND              |
| RST             | Navigation buttons' reset pin                                | (Same as left)                                               | RST              |
| GND             | Multiple modules' GND (GPS, LORA, Lidar, OLED, PCF8574, ALAMSCN, etc.) | Free                                                         | RX (D0)          |
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

**Note:** One pair of the ALAMSCN IR receiver and transmitter mapped here. The receiver’s **DAT** pin connects to Nano A2 (input), and the transmitter’s **DAT** pin connects to Nano A3 (output).

### Modules - Sensors

#### Magnetic Sensor

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

#### BME280

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

#### Lidar Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| TX         | D4       |
| RX         | D5       |

**Note:** Uses SoftwareSerial on pins D4 (RX) and D5 (TX).

#### RTC Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

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
| reset      | Nano RST                   |

**Note:** One navigation module mapped, connected to a single PCF8574 expander via the I2C bus (A4 → SDA, A5 → SCL).

#### Universal 4 Key Push Button Switch Module

| Module Pin | Connection                 |
|------------|----------------------------|
| GND        | GND                        |
| VCC        | 5V                         |
| Key1       | PCF8574 P0 (I2C on A4, A5) |
| Key2       | PCF8574 P1 (I2C on A4, A5) |
| Key3       | PCF8574 P2 (I2C on A4, A5) |
| Key4       | PCF8574 P3 (I2C on A4, A5) |

**Note:** Connected to a separate PCF8574 expander with a unique I2C address, freeing up Nano pins A0–A3.

#### PCF8574

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** One PCF8574 module mapped here, connected to the I2C bus on Nano pins A4 (SDA) and A5 (SCL). Multiple PCF8574s are used (e.g., for navigation buttons and 4-key module), each with a unique I2C address.

#### OLED Module

| Module Pin | Nano Pin |
|------------|----------|
| VCC        | 5V       |
| GND        | GND      |
| SDA        | A4       |
| SCL        | A5       |

**Note:** Connected to the I2C bus on pins A4 (SDA) and A5 (SCL).

### Modules - Bulb

#### Red/Green Bi-Colour LED (Pack of 10)

| Module Pin       | Nano Pin |
|------------------|----------|
| Red Anode        | D6 (via 220Ω resistor) |
| Green Anode      | D7 (via 220Ω resistor) |
| Common Cathode   | GND      |

**Note:** One LED mapped here, connected to D6 and D7 via 220Ω resistors.

#### Red/Green Bi-Colour LED (Additional)

| Module Pin       | Nano Pin |
|------------------|----------|
| Red Anode        | D8 (via 220Ω resistor) |
| Green Anode      | D9 (via 220Ω resistor) |
| Common Cathode   | GND      |

**Note:** One additional LED mapped here, connected to D8 and D9 via 220Ω resistors.

#### Red/Green Bi-Colour LED (Linked to Nano Light)

| Module Pin       | Nano Pin |
|------------------|----------|
| Red Anode        | D13 (via 220Ω resistor) |
| Common Cathode   | GND      |

**Note:** One LED linked to the Nano’s onboard LED on D13, using a 220Ω resistor. Mirrors the onboard LED’s state.

#### 220 ohm Resistor

| Module Pin | Nano Pin |
|------------|----------|
| N/A        | N/A      |

**Note:** Resistors are used in series with LED anodes (D6-D9, D13) as specified above.

### Modules - Power

#### AZDelivery 18650 Lithium Li-ion Battery Expansion Shield 5V – 3V Micro USB Module

| Module Pin | Nano Pin |
|------------|----------|
| 5V Out     | 5V       |
| 3V Out     | 3.3V     |
| GND        | GND      |
| Micro USB  | VIN      |

**Note:** This module provides power to the Nano. The **5V Out** connects to the Nano’s 5V pin, **3V Out** to the 3.3V pin if needed, and **GND** to any Nano GND pin. The **Micro USB** port connects to VIN for external 5V input.

---

### General Notes

- **I2C Bus:** Multiple modules (OLED, PCF8574 for navigation buttons, PCF8574 for 4-key module, Magnetic Sensor, BME280, RTC) share the I2C bus on Nano pins **A4 (SDA)** and **A5 (SCL)**. Ensure each PCF8574 has a unique I2C address (configurable via address pins A0–A2 on the PCF8574 board) to avoid conflicts.
- **ALAMSCN IR Module Updates:**
  - **Receiver:** Moved from **D12** to **A2** (input).
  - **Transmitter:** Remains on **A3** (output).
- **Universal 4 Key Module Update:** Connected to a **PCF8574 expander** using pins **P0–P3**, freeing up Nano pins A0–A3.
- **Pin Availability:**
  - **D0 (RX)** and **D1 (TX)** are free.
  - **A0–A1** are free and can be used as digital I/O or analog inputs if needed.
  - **A6–A7** remain free but are analog input-only pins (can be used as digital inputs, not outputs).
  - **D12:** Now free after moving the ALAMSCN IR Receiver DAT to A2.
- **D13 Usage:** Connected to an LED mirroring the Nano’s onboard LED, with a 220Ω resistor limiting current to ~13mA, keeping the total draw (onboard + external LED) within safe limits (~20mA).

---

This updated `guild.md` reflects the change of moving the **ALAMSCN IR Receiver DAT** from **D12** to **A2**, with all tables and notes adjusted accordingly. The document is consistent, and pin assignments have been verified to avoid conflicts. You can copy this directly into your `.md` file. Let me know if you need further adjustments!