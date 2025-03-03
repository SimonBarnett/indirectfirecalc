# Installing Code on Arduino Nano for RF-Minimized Spotter System

This guide outlines the steps to upload code to an Arduino Nano for the RF-Minimized Weapon-Mounted Spotter System devices (S1/S2, B1, W1) using the Arduino IDE.

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