# LOW-VOLTAGE-AC-DRIVES-ACS580-Or-ACS880-ESP32-control-Driver-using-PLC-and-Profinet-Protocol

## Project Overview

This project enables the control of a LOW VOLTAGE AC DRIVE (specifically ABB ACS580 or ACS880 models) using an Espressif ESP32 microcontroller and a SIMATIC ET 200SP PLC. Communication between the ESP32 and the PLC is established using the Profinet protocol. The system also integrates a servo motor (SG-90) for physical actuation and an ultrasonic distance sensor (HC-SR04) for environmental sensing.

The primary purpose is to provide a flexible and modern interface for controlling industrial AC drives, allowing for remote monitoring and automation tasks.

Key Features:
*   Remote control of ABB ACS580/ACS880 AC drives.
*   Integration with Siemens SIMATIC ET 200SP PLC via Profinet.
*   Utilizes ESP32 for processing and network communication.
*   Includes servo motor control and ultrasonic distance sensing.
*   Allows for both WiFi and potentially Ethernet-based communication for the ESP32.

## Hardware Setup

The following components are required for this project:

*   **LOW VOLTAGE AC DRIVES:**
    *   ABB general purpose drives: ACS580 or ASC880 (0.75 to 500 kW)
*   **Microcontroller:**
    *   Espressif ESP32
*   **Actuator:**
    *   Servo Motor SG-90
        *   Connection: Typically connected to ESP32 GPIO pin 15.
*   **Sensor:**
    *   Ultrasonic Distance Sensor - HC-SR04
        *   Connections:
            *   Trigger Pin (Trig): ESP32 GPIO pin 27
            *   Echo Pin (Echo): ESP32 GPIO pin 14
*   **Relay Module:**
    *   5 Volt 4 Channel Arduino Relay Module
*   **PLC:**
    *   SIMATIC ET 200SP
*   **Network Hardware:**
    *   WiFi Internet Connection hardware (e.g., wireless router)
    *   (Optional) Ethernet Shield for ESP32 if not using WiFi.

## Software Setup

### Recommended IDE

*   **Arduino IDE**

### Board Configuration

*   **Board:** ESP32
    *   Ensure you have added ESP32 board support to your Arduino IDE. You can typically do this through the Board Manager. Search for "esp32" and install the package by Espressif Systems.

### Required Libraries

Install the following libraries through the Arduino Library Manager or by manual installation:

*   **Settimino:** For ARDUINO™ Ethernet/WiFi communication with S7 Siemens™ PLCs.
    *   Link: `http://settimino.sourceforge.net/`
    *   Installation: This library might require manual installation. Follow the instructions on the Settimino website.
*   **HC-SR04:** For the ultrasonic distance sensor.
    *   Link: `https://github.com/d03n3rfr1tz3/HC-SR04`
    *   Installation: Can usually be found in the Arduino Library Manager by searching "HCSR04".
*   **ESP32Servo:** For controlling the servo motor with ESP32.
    *   Link: `https://github.com/jkb-git/ESP32Servo`
    *   Installation: Search "ESP32Servo" in the Arduino Library Manager.

### Network Configuration

You will need to update the network settings within the main `.ino` sketch file. Specifically, modify the following placeholders with your actual network details:

```cpp
// Example placeholder - actual variables might differ in your sketch
char ssid[] = "YOUR_WIFI_SSID";       // your network SSID (name)
char pass[] = "YOUR_WIFI_PASSWORD";   // your network password

// IP Addresses - ensure these are correct for your network configuration
byte local_ip[] = {192, 168, 1, 100}; // ESP32 IP Address
byte plc_ip[]   = {192, 168, 1, 200}; // PLC IP Address
byte gateway[]  = {192, 168, 1, 1};   // your network gateway (router)
byte subnet[]   = {255, 255, 255, 0}; // your network subnet mask
```

## How it Works

1.  **Network Initialization:**
    *   The ESP32 connects to the local network using either WiFi (credentials provided in the code) or an Ethernet connection (if an Ethernet shield is used and configured).

2.  **PLC Communication:**
    *   The Settimino library is used to establish communication with the Siemens SIMATIC ET 200SP PLC over the Profinet protocol.
    *   The ESP32 reads and writes data to a specific Data Block (DB) on the PLC. By default, this is often DB2, but it's configurable in the sketch.

3.  **Sensor Integration:**
    *   **Ultrasonic Sensor (HC-SR04):** The ESP32 reads distance measurements from the HC-SR04 sensor.
    *   **Analog Input (Potentiometer):** An analog value is read from a potentiometer connected to ESP32 GPIO pin 34. This value is typically used for frequency control of the AC drive.

4.  **Actuator Control:**
    *   **Servo Motor (SG-90):** The ESP32 controls the position of the SG-90 servo motor. The servo's movement can be triggered or controlled by data received from the PLC.

5.  **Pin Functions (Example):**
    The sketch utilizes various GPIO pins on the ESP32 for different functions. Key pins might include:
    *   `marche`: Input/Output for starting/stopping the drive.
    *   `lampe`: Output for an indicator light.
    *   `S3`, `S4`: Digital inputs for operator controls.
    *   `Ready`: Input indicating drive readiness.
    *   `defaut`: Input indicating a drive fault.
    *   `thermistors`: Input for motor thermistor monitoring.
    *   `inuse`: Output indicating the drive is active.
    *   `frequencepot`: Analog input (pin 34) for frequency setpoint via potentiometer.
    *   Pin 27 (Trig) & 14 (Echo) for HC-SR04.
    *   Pin 15 for Servo Motor.

## Troubleshooting

Here are some common issues and how to address them:

*   **WiFi/Ethernet Connection Issues:**
    *   Double-check WiFi SSID and password in the sketch.
    *   Verify the IP addresses (`local_ip`, `plc_ip`, `gateway`, `subnet`) are correctly configured for your network.
    *   Ensure the ESP32 has a good signal or a proper physical Ethernet connection.
    *   Check router/switch configurations for any blocking rules.
*   **PLC Communication Errors:**
    *   Confirm the PLC's IP address is correct in the sketch.
    *   Ensure the Data Block number (e.g., DB2) used in the sketch matches the one configured and available on the PLC.
    *   Check network cabling between the ESP32 (or its Ethernet interface) and the PLC.
    *   Verify Profinet communication is enabled and configured correctly on the PLC.
*   **Sensor Reading Inaccuracies:**
    *   **HC-SR04:** Check wiring to trigger and echo pins. Ensure a clear path for the ultrasonic waves. The sensor might be faulty.
    *   **Potentiometer:** Verify wiring to the analog input pin (e.g., GPIO 34). Check if the potentiometer itself is functioning correctly.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes or enhancements.

## License

This project is licensed under the MIT License. It is recommended to create a `LICENSE` file in the root of the repository with the contents of the MIT License.
```
You can find the text of the MIT License here: https://opensource.org/licenses/MIT
```

## Disclaimer
The information provided in this document is for guidance and informational purposes only. Working with electrical equipment, especially high-voltage AC drives and PLCs, carries inherent risks. Always ensure you have the necessary knowledge, skills, and safety precautions in place before attempting to replicate or modify this project. The authors are not responsible for any damage, injury, or loss resulting from the use or misuse of this information. Always consult the official documentation for the hardware components used.
