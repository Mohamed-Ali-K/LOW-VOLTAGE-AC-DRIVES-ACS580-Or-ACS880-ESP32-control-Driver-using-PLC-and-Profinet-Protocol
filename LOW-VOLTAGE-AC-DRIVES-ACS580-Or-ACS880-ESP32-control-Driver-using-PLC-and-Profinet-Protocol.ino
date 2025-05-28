/*
 * Project: ESP32 Control for Low-Voltage AC Drives (ACS580/ACS880) via PLC and Profinet
 * File: LOW-VOLTAGE-AC-DRIVES-ACS580-Or-ACS880-ESP32-control-Driver-using-PLC-and-Profinet-Protocol.ino
 * Author: Your Name / Company
 * Date: 2023-10-27
 * Version: 1.0.1
 *
 * Description:
 * This sketch enables an ESP32 microcontroller to control ABB ACS580 or ACS880
 * low-voltage AC drives. It communicates with a Siemens SIMATIC ET 200SP PLC
 * using the Profinet protocol via the Settimino library. The system integrates
 * an HC-SR04 ultrasonic sensor for distance measurement and an SG-90 servo motor
 * for control, potentially based on PLC commands or sensor feedback. It also
 * reads an analog input (potentiometer) for frequency setpoint.
 *
 * Hardware:
 * - ESP32 Development Board
 * - ABB ACS580/ACS880 AC Drive
 * - Siemens SIMATIC ET 200SP PLC (or other S7 compatible PLC)
 * - HC-SR04 Ultrasonic Distance Sensor
 * - SG-90 Servo Motor
 * - Potentiometer (e.g., 10k ohm) for analog input
 * - Relays, LEDs, and other I/O components as defined by pin connections.
 *
 * Libraries:
 * - Settimino: For S7 PLC communication over Ethernet/Profinet. (http://settimino.sourceforge.net/)
 * - HCSR04: For interfacing with the HC-SR04 ultrasonic sensor. (e.g., https://github.com/d03n3rfr1tz3/HC-SR04)
 * - ESP32Servo: For controlling servo motors with ESP32. (e.g., https://github.com/jkb-git/ESP32Servo)
 */

#include "Platform.h" // Platform specific definitions for Settimino, ensures compatibility across different Arduino boards/platforms.
#include "Settimino.h"  // Core library for Siemens S7 PLC communication (Profinet/Ethernet TCP/IP).
#include <HCSR04.h>   // Library for interfacing with the HC-SR04 ultrasonic distance sensor.
#include <ESP32Servo.h> // Library providing servo motor control functions specifically for ESP32.

// If defined, Settimino uses a smaller internal buffer (PDU.DATA, typically around 25 bytes based on S7 default PDU size) for PLC communication.
// If undefined (commented out), a larger external buffer ('Buffer' variable, 1024 bytes here) would be used for operations like ReadArea/WriteArea if 'Target' points to it.
#define DO_IT_SMALL

// Global Variables:

// Servo and Sensor Objects:
Servo myservo;  // Create a servo object to control an SG-90 servo motor.
                // This object will be used to command the servo to specific angles using myservo.write().
HCSR04 hc(27,14); // Initialize an HC-SR04 Ultrasonic Sensor object.
                  // Pin 27 is configured as the Trigger Pin (Trig) for sending ultrasonic pulses.
                  // Pin 14 is configured as the Echo Pin (Echo) for receiving reflected pulses.

// Network Configuration Parameters:
IPAddress Local(192,168,43,90);   // Static IP address assigned to this ESP32 module on the local network.
IPAddress PLC(192,168,43,52);     // Static IP address of the Siemens PLC that the ESP32 will communicate with.
char ssid[] = "Dar internen";      // WiFi Network SSID (Service Set Identifier) - the name of your wireless network.
char pass[] = "00000003";    // WiFi Network Password - the password for your wireless network.
IPAddress Gateway(192, 168,43, 1); // IP address of your network gateway (typically your router's IP address).
IPAddress Subnet(255, 255, 255, 0);  // Subnet mask for the local network. This defines the range of IP addresses in your local network.

// PLC Communication Parameters:
int DBNum = 2;              // Data Block number in the PLC to be read from/written to. This DB must exist in the PLC project.
byte Buffer[1024];          // Data buffer for PLC communication. Used when DO_IT_SMALL is NOT defined and Target points to this buffer.
                            // Max size 1024 bytes. For larger data exchanges.
S7Client Client;            // S7Client object from the Settimino library; handles the communication protocol with the S7 PLC.
unsigned long Elapsed;      // Variable to store elapsed time (in milliseconds) for profiling communication or other operations. Used by MarkTime() and ShowTime().

// Sensor and Control Variables:
int niveau;                 // Stores the distance measurement (typically in cm, after conversion) from the HC-SR04 ultrasonic sensor.
float MfrequanceF = 14;     // Floating-point variable, likely related to frequency. Its specific use is not prominent in the current loop logic,
                            // possibly a remnant or for future development. Defaulted to 14.0.
int MfrequanceI = 15;       // Integer variable, primarily used to store the frequency value read from the PLC (at DB2, offset 14).
                            // This value is then mapped to control the servo motor's position. Defaulted to 15.
int servoPin = 15;          // ESP32 GPIO pin connected to the servo motor's signal wire. (Currently GPIO 15)
int times = 15000 ;         // Time-related variable, possibly intended for a delay or timeout in milliseconds.
                            // However, it's not actively used in the current loop() logic. Defaulted to 15000.
int frequenceval;           // Stores the raw analog reading (0-4095) from the 'frequencepot' potentiometer.
                            // It is then mapped to a frequency range (e.g., 0-50 Hz) and sent to the PLC.

// Pin Definitions (ESP32 GPIOs):
int LED_BUILTIN = 2;        // GPIO pin for the ESP32's built-in LED (usually GPIO 2 on many ESP32 boards). Used for status indication (e.g., PLC connection).
const int marche = 26;      // Digital output pin to control the start/stop (march/run) command for the AC drive.
                            // Typically, LOW might mean Run and HIGH might mean Stop, depending on drive configuration. (GPIO 26)
const int lampe = 25;       // Digital output pin for an external indicator lamp. (GPIO 25)
                            // The lamp's state (ON/OFF for LOW/HIGH) depends on how it's wired.
const int S3 = 33;          // Digital output pin, likely for an operator control panel button feedback or signal S3. (GPIO 33)
const int S4 = 32;          // Digital output pin, likely for an operator control panel button feedback or signal S4. (GPIO 32)
const int Ready = 21;       // Digital input pin to read the 'Ready' status from the AC drive. (GPIO 21)
                            // (Configured with INPUT_PULLUP, so LOW means drive is ready, HIGH means not ready).
const int defaut = 19;      // Digital input pin to read the 'Fault' status from the AC drive. (GPIO 19)
                            // (Configured with INPUT_PULLUP, so LOW means a fault is active, HIGH means no fault).
const int thermistors = 18; // Digital input pin to monitor motor thermistors status (overheating protection). (GPIO 18)
                            // (Configured with INPUT_PULLUP, so LOW means thermistor trip/fault, HIGH means okay).
const int inuse = 5;        // Digital input pin to check if the AC drive is 'In Use' or running. (GPIO 5)
                            // (Configured with INPUT_PULLUP, so LOW means drive is in use, HIGH means not in use).
int frequencepot = 34;      // Analog input pin (ADC1_CH6 on ESP32) connected to a potentiometer for frequency setpoint adjustment.
int ADC_Max = 4095;         // Maximum value for the ESP32's ADC (12-bit resolution: 2^12 - 1 = 4095). Used for mapping the analog input.
uint16_t count = 0;         // A counter variable. Its specific use is not evident in the current main loop() logic,
                            // might be for future features, debugging, or counting specific events.

// Initialization function, runs once when the ESP32 starts up.
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate for debugging and outputting information to the Serial Monitor.

#ifdef S7WIFI
//--------------------------------------------- ESP32 WiFi Initialization Block ---------------------------------------------
    Serial.println(); // Print a blank line for better readability in Serial Monitor.
    Serial.print("Connecting to "); // Print connection message.
    Serial.println(ssid); // Print the SSID (network name) the ESP32 is trying to connect to.
    WiFi.begin(ssid, pass); // Initiate WiFi connection with the provided SSID and password.
    WiFi.config(Local, Gateway, Subnet); // Configure the ESP32 with a static IP address, gateway, and subnet mask.
                                         // Using a static IP is often preferred for reliable PLC communication.
    while (WiFi.status() != WL_CONNECTED) // Loop until the WiFi connection is established.
    {
        delay(500);        // Wait for 500 milliseconds.
        Serial.print("."); // Print a dot to indicate the connection process is ongoing.
    }
    Serial.println(""); // New line after successful connection.
    Serial.println("WiFi connected");  // Print confirmation message.
    Serial.print("Local IP address : ");
    Serial.println(WiFi.localIP()); // Print the IP address assigned to the ESP32.
#else
//--------------------------------Wired Ethernet Shield Initialization Block (Alternative, if S7WIFI is not defined) --------------------------------
    // This block would contain initialization code for a wired Ethernet shield (e.g., W5x00 series using Ethernet.h).
    // It typically requires defining a MAC address and might involve setting up the CS pin.
    // Example (ensure you have the Ethernet library included and MAC address defined):
    // byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    // Ethernet.init(YOUR_CS_PIN); // If your Ethernet shield needs a specific CS pin.
    // Ethernet.begin(mac, Local, Gateway, Subnet); // Initialize Ethernet with static IP configuration.
    
    // The line below is a placeholder from the original code.
    // EthernetInit(mac, Local); // This function `EthernetInit` is not standard in Arduino Ethernet library.
                               // It implies a custom function or a specific version of Settimino that includes it.
                               // For standard Ethernet library, use Ethernet.begin().
    
    // Comment from original code, refers to a potential delay needed for some Ethernet shields to initialize.
    delay(2000); // Delay to allow Ethernet shield to initialize properly.
    Serial.println("");
    Serial.println("Cable connected");  // Print confirmation message (assuming Ethernet connection is attempted).
    Serial.print("Local IP address : ");
    // Serial.println(Ethernet.localIP()); // Print the IP address assigned via Ethernet. (Requires Ethernet library and successful init)
#endif

  // Servo Motor Configuration:
  myservo.setPeriodHertz(50); // Set the PWM frequency for the servo to 50Hz. This is standard for most analog servos like the SG-90.
  myservo.attach(servoPin, 500, 2400); // Attach the servo object to the 'servoPin' (GPIO 15).
                                       // Sets the minimum (500 microseconds) and maximum (2400 microseconds) pulse widths.
                                       // These values are common for SG90 servos to achieve approximately 0-180 degree range of motion.
                                       // For other servos (e.g., MG995), these might be 1000us (min) and 2000us (max) or other values.
  
  // Initialize Digital Output Pin States (Setting initial safe states before the main loop begins):
  digitalWrite(marche, HIGH); // Set 'marche' pin HIGH. Assuming HIGH corresponds to the drive being stopped or in a safe default state.
  digitalWrite(lampe, HIGH);  // Set 'lampe' pin HIGH. Assuming HIGH means the lamp is initially OFF.
  digitalWrite(S3, HIGH);     // Set 'S3' output pin HIGH initially.
  digitalWrite(S4, HIGH);     // Set 'S4' output pin (GPIO 32 as per 'S4' const) HIGH initially.

  // Configure Pin Modes for ESP32 GPIOs:
  pinMode(marche, OUTPUT);        // Configure 'marche' (GPIO 26) as an OUTPUT pin (to control AC drive start/stop command).
  pinMode(lampe, OUTPUT);         // Configure 'lampe' (GPIO 25) as an OUTPUT pin (to control an indicator lamp).
  pinMode(S3, OUTPUT);            // Configure 'S3' (GPIO 33) as an OUTPUT pin.
  pinMode(S4, OUTPUT);            // Configure 'S4' (GPIO 32) as an OUTPUT pin.
  pinMode(LED_BUILTIN, OUTPUT);   // Configure 'LED_BUILTIN' (GPIO 2) as an OUTPUT pin (for onboard status LED).
  
  // Configure digital input pins with internal pull-up resistors.
  // This means if the external signal is disconnected, the pin will read HIGH.
  // An active LOW signal from the drive/sensor will pull the pin LOW.
  pinMode(Ready, INPUT_PULLUP);   // Configure 'Ready' (GPIO 21) as an INPUT with an internal pull-up resistor.
                                  // (Reads AC drive 'Ready' status; LOW if ready, HIGH if not ready/disconnected).
  pinMode(defaut, INPUT_PULLUP);  // Configure 'defaut' (GPIO 19) as an INPUT with an internal pull-up resistor.
                                  // (Reads AC drive 'Fault' status; LOW if fault is active, HIGH if no fault/disconnected).
  pinMode(thermistors, INPUT_PULLUP); // Configure 'thermistors' (GPIO 18) as an INPUT with an internal pull-up resistor.
                                      // (Monitors motor thermistors; LOW if thermistor trip/overheat, HIGH if okay/disconnected).
  pinMode(inuse, INPUT_PULLUP);   // Configure 'inuse' (GPIO 5) as an INPUT with an internal pull-up resistor.
                                  // (Checks if AC drive is 'In Use'; LOW if in use, HIGH if not in use/disconnected).
  
  // Note: frequencepot (GPIO 34) is an Analog-to-Digital Converter (ADC) pin.
  // pinMode() is not strictly required for analogRead() on ESP32, as pins are ADC capable by default if they support it.
}

// Function to establish a connection with the Siemens S7 PLC.
// Returns true if connection is successful, false otherwise.
bool Connect()
{
    // Attempt to connect to the PLC using its IP address, Rack number, and Slot number.
    // These parameters are specific to the Siemens S7 PLC configuration.
    // - PLC: The IPAddress object of the target PLC.
    // - Rack number: For S7-1200/1500 series, Rack is typically 0. For S7-300/400, it's also often 0.
    // - Slot number: For S7-1200/1500, Slot is often 0 or 1 (for the integrated Profinet interface of the CPU).
    //                For S7-300/400 with a separate Communications Processor (CP), the slot of the CP is used (e.g., 2).
    //                Consult your PLC's hardware configuration in TIA Portal or Simatic Manager.
    int Result=Client.ConnectTo(PLC, 0, 2); // Current parameters: Rack 0, Slot 2.
                                         
    Serial.print("Connecting to PLC at IP: ");Serial.println(PLC); // Log connection attempt.
    if (Result==0) // Check if the connection was successful (Result code 0 from Settimino means success).
    {
      Serial.print("Connected to PLC! PDU Length negotiated = ");Serial.println(Client.GetPDULength()); // Log success and the negotiated PDU (Protocol Data Unit) length.
    }
    else
    {
      Serial.print("Connection to PLC failed. Error code: 0x"); Serial.println(Result, HEX); // Log failure and the error code from Settimino in hexadecimal.
    }
    return Result==0; // Return true if connected successfully (Result is 0), false otherwise.
}

// Function to check for and print PLC communication errors.
// It also disconnects the S7Client if a severe error occurs.
void CheckError(int ErrNo)
{
  Serial.print("PLC Communication Error. Code: 0x"); // Print the error number in hexadecimal format for diagnostics.
  Serial.println(ErrNo, HEX);
  
  // Checks if it's a "Severe Error". The condition (ErrNo & 0x00FF) likely isolates
  // specific error codes defined by the Settimino library that are considered critical and unrecoverable,
  // often indicating a lost connection or major configuration issue.
  if (ErrNo & 0x00FF) // This is a bitwise AND; if any bit in the lower byte of ErrNo is set, it's treated as severe.
  {
    Serial.println("SEVERE ERROR DETECTED. Disconnecting from PLC.");
    Client.Disconnect(); // Disconnect the S7Client from the PLC to allow for a clean reconnection attempt later.
  }
}

//----------------------------------------------------------------------
// Profiling routines: Used to measure the execution time of code segments.
//----------------------------------------------------------------------

// Records the current time (in milliseconds since the ESP32 started).
// This function should be called before the code segment to be measured.
void MarkTime()
{
  Elapsed=millis(); // `millis()` returns the number of milliseconds passed since the ESP32 board began running the current program.
                    // This value is stored in the global 'Elapsed' variable.
}

// Calculates and prints the time elapsed since MarkTime() was last called.
// This function should be called after the code segment to be measured.
void ShowTime()
{
  unsigned long currentTime = millis(); // Get the current time.
  Elapsed = currentTime - Elapsed;    // Calculate the difference between current time and the time stored in 'Elapsed'.
  Serial.print("Operation duration (ms): ");
  Serial.println(Elapsed);   // Print the calculated duration in milliseconds.
}

// Main program loop, runs repeatedly after setup() is complete.
void loop() {
  
  // Read status from digital input pins connected to AC drive signals.
  // These pins are configured with INPUT_PULLUP, so a LOW signal means the condition is active (e.g., drive is Ready).
  int ReadyVal = digitalRead(Ready);             // Read 'Ready' status from AC drive. (LOW = Ready, HIGH = Not Ready)
  int defautVal = digitalRead(defaut);           // Read 'Fault' status from AC drive. (LOW = Fault Active, HIGH = No Fault)
  int thermistorsVal = digitalRead(thermistors); // Read motor thermistor status. (LOW = Thermistor Fault/Overheat, HIGH = OK)
  int inuseVal = digitalRead(inuse);             // Read 'In Use' status of AC drive. (LOW = In Use, HIGH = Not In Use)
  
  // int frequance ; // This variable is declared but not used in the loop. It can be removed to save memory and avoid confusion.
  
  // Read distance from HC-SR04 ultrasonic sensor.
  niveau = int(hc.dist()); // hc.dist() likely returns a float value representing distance (e.g. in cm). It's cast to an integer here.
  
  // Read analog input from potentiometer (for frequency setpoint).
  frequenceval = analogRead(frequencepot); // Read raw ADC value (0-4095 for 12-bit ESP32 ADC) from 'frequencepot' (GPIO 34).
  Serial.print("Raw Potentiometer ADC value: "); Serial.println(frequenceval); // Print raw ADC value for debugging/calibration.
  
  // Map the ADC value (0-ADC_Max, typically 0-4095) to a frequency range (e.g., 0-50 Hz).
  // This allows the potentiometer position to control the target frequency sent to the PLC.
  frequenceval = map(frequenceval, 0, ADC_Max, 0, 50); // map(value, fromLow, fromHigh, toLow, toHigh)

  // Variables for PLC communication.
  int Size;   // Holds the size of the data packet (in bytes) to be read/written to the PLC.
  int Result; // Stores the result code of PLC communication operations (0 for success from Settimino, others for errors).
  void *Target; // Pointer to the data buffer for PLC communication.
                // If DO_IT_SMALL is defined, Target is NULL, and Settimino uses its internal buffer (PDU.DATA[]).
                // If DO_IT_SMALL is not defined, Target points to the global 'Buffer' array.
  
  // Variables to store data read from PLC (control signals from PLC to ESP32).
  bool P60, Rege, D3, D4; // Boolean flags based on bits read from PLC DB2.
                           // P60: Likely controls 'marche' (run/stop command for the drive).
                           // Rege: Likely controls 'lampe' (an indicator light, possibly for regeneration status or general purpose).
                           // D3, D4: Likely control digital outputs S3 and S4.

#ifdef DO_IT_SMALL
  Size=25;        // If DO_IT_SMALL is defined, set data packet size for read/write operations to 25 bytes.
                  // This should be coordinated with the actual data size configured in the PLC's Data Block.
  Target = NULL;  // Use Settimino's internal PDU buffer (PDU.DATA[]). This buffer is part of the S7Client object.
                  // Suitable for small data exchanges.
#else
  Size=1024;      // If DO_IT_SMALL is not defined (commented out), set data packet size to 1024 bytes.
  Target = &Buffer; // Use the globally declared 'Buffer' array. Suitable for larger data exchanges.
#endif
  
  // PLC Connection Management: Ensure connection to PLC is active before proceeding with read/write operations.
  if (!Client.Connected) // Check if the client is currently connected.
  {
    Serial.println("PLC not connected. Attempting to connect...");
    if (!Connect()) // Attempt to connect to the PLC by calling the Connect() function.
    {
      delay(500);   // If connection fails, wait for 500 milliseconds before the loop repeats and tries again.
      return;       // Skip the rest of the loop if connection failed.
    }
  }

  // Indicate PLC connection status with the built-in LED.
  if (Client.Connected)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn LED ON (HIGH is usually ON for ESP32 built-in LED) if connected.
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);  // Turn LED OFF if not connected.
  }
  
  // --- Write data from ESP32 (sensor statuses, setpoints) to the PLC Data Block (DBNum) ---
  // The S7.SetBitAt and S7.SetIntAt functions prepare data in the 'Target' buffer (or PDU.DATA if Target is NULL).
  // Client.WriteArea then sends this prepared data to the specified PLC DB address.
  // IMPORTANT: When Target is NULL (DO_IT_SMALL defined), S7.SetBitAt/SetIntAt use an internal buffer in Settimino (PDU.DATA).
  // Each call to S7.SetBitAt(Target, byteOffset, bitOffset, value) or S7.SetIntAt(Target, byteOffset, value)
  // modifies this *same* internal buffer starting at 'byteOffset' within that buffer.
  // So, to write different values to different PLC addresses, you must:
  // 1. Prepare data in the buffer using S7.Set...(Target, internal_buffer_offset, value).
  // 2. Call Client.WriteArea(..., plc_db_offset, size_of_data, Target) for that specific data.
  // 3. Repeat for other data items, ensuring 'internal_buffer_offset' is 0 if you're writing one item at a time.

  byte singleByteData; // Temporary byte buffer for writing single bit values.

  // Write 'Ready' status to PLC (e.g., DB2, Byte 0, Bit 0)
  // Logic: If ReadyVal is HIGH (drive NOT ready due to PULLUP), PLC bit 0.0 is set to 0.
  //        If ReadyVal is LOW (drive IS ready), PLC bit 0.0 is set to 1.
  singleByteData = 0; // Clear the byte
  if (ReadyVal == LOW) { // Drive is Ready
    bitSet(singleByteData, 0); // Set bit 0 of singleByteData to 1
  } else { // Drive is Not Ready
    bitClear(singleByteData, 0); // Set bit 0 of singleByteData to 0
  }
  // Write 1 byte (singleByteData) to DBNum, starting at PLC byte offset 0.
  Client.WriteArea(S7AreaDB, DBNum, 0, 1, &singleByteData);

  // Write 'Thermistors' status to PLC (e.g., DB2, Byte 0, Bit 1, if it were in the same byte)
  // Original code writes to PLC DB offset 8 for thermistors.
  // Logic: If thermistorsVal is HIGH (no fault), PLC bit (at offset 8.0) is 0.
  //        If thermistorsVal is LOW (fault), PLC bit (at offset 8.0) is 1.
  //        Also, if thermistor fault, force 'marche' output HIGH (stop drive) and write 0 to PLC offset 20.0.
  singleByteData = 0; // Clear the byte
  if (thermistorsVal == LOW) { // Thermistor fault
    digitalWrite(marche, HIGH); // Stop the drive
    bitSet(singleByteData, 0); // Set bit for thermistor fault (e.g. bit 0 of this byte)
    
    // Write fault status to PLC DB offset 8
    Client.WriteArea(S7AreaDB, DBNum, 8, 1, &singleByteData);
    
    // Additionally, set bit 0 of byte 0 (in a new temporary byte) to 0, and write to PLC DB offset 20.
    byte bit20_data = 0; // Data for PLC offset 20
    bitClear(bit20_data, 0); // Clear bit 0 (set to 0)
    Client.WriteArea(S7AreaDB, DBNum, 20, 1, &bit20_data); // Write 1 byte to offset 20 of DBNum.
  } else { // No thermistor fault
    bitClear(singleByteData, 0); // Clear bit for thermistor fault
    Client.WriteArea(S7AreaDB, DBNum, 8, 1, &singleByteData); // Write 1 byte to offset 8 of DBNum.
  }

  // Write 'Defaut' (Fault) status to PLC (e.g., DB2, Byte 1, Bit 4)
  // Original code writes to PLC DB offset 12.
  // Logic: If defautVal is HIGH (no fault), PLC bit (at offset 12.0) is 0.
  //        If defautVal is LOW (fault), PLC bit (at offset 12.0) is 1.
  singleByteData = 0; // Clear the byte
  if (defautVal == LOW) { // Drive fault active
    bitSet(singleByteData, 0); // Set bit for drive fault
  } else { // No drive fault
    bitClear(singleByteData, 0); // Clear bit for drive fault
  }
  Client.WriteArea(S7AreaDB, DBNum, 12, 1, &singleByteData); // Write 1 byte to offset 12 of DBNum.

  // Write 'In Use' status to PLC (e.g., DB2, Byte 0, Bit 2)
  // Original code writes to PLC DB offset 4.
  // Logic: If inuseVal is HIGH (not in use), PLC bit (at offset 4.0) is 0.
  //        If inuseVal is LOW (in use), PLC bit (at offset 4.0) is 1.
  singleByteData = 0; // Clear the byte
  if (inuseVal == LOW) { // Drive is in use
    bitSet(singleByteData, 0); // Set bit for in-use status
  } else { // Drive is not in use
    bitClear(singleByteData, 0); // Clear bit for in-use status
  }
  Client.WriteArea(S7AreaDB, DBNum, 4, 1, &singleByteData);  // Write 1 byte to offset 4 of DBNum.

  // Write sensor data (distance - 'niveau') and frequency setpoint ('frequenceval') to PLC.
  // For these, S7.SetIntAt will prepare the 2-byte integer in the Target buffer (PDU.DATA if Target is NULL).
  // The first argument to SetIntAt is the offset *within the Target buffer*.
  
  // Prepare 'niveau' (integer distance) to be written.
  S7.SetIntAt(Target, 0, niveau); // Place 'niveau' into the first two bytes (Word) of Target buffer (e.g., PDU.DATA[0] and PDU.DATA[1]).
  // Write these 2 bytes from Target buffer to DBNum at PLC byte offset 18.
  Client.WriteArea(S7AreaDB, DBNum, 18, 2, Target); 

  // Prepare 'frequenceval' (mapped potentiometer frequency) to be written.
  S7.SetIntAt(Target, 0, frequenceval); // Place 'frequenceval' into the first two bytes of Target buffer (overwriting previous 'niveau' if Target is PDU.DATA and offset is 0).
  // Write these 2 bytes from Target buffer to DBNum at PLC byte offset 6.
  Client.WriteArea(S7AreaDB, DBNum, 6, 2, Target);
  
  // --- Read data from PLC Data Block (DBNum) ---
  Serial.print("Reading ");Serial.print(Size);Serial.print(" bytes from DB");Serial.print(DBNum); Serial.print(" starting at offset 0."); Serial.println();
  MarkTime(); // Start timing the PLC read operation.
  
  // Read 'Size' bytes from PLC DBNum, starting at byte offset 0, into 'Target' buffer (or PDU.DATA if Target is NULL).
  Result=Client.ReadArea(S7AreaDB, // Area code for Data Blocks.
                         DBNum,    // DB Number to read from.
                         0,        // Starting byte offset within the PLC DB.
                         Size,     // Number of bytes to read from the PLC DB.
                         Target);  // Pointer to buffer where data read from PLC will be stored.
  
  if (Result==0) // If PLC read operation is successful (Result code 0 from Settimino).
  {
    ShowTime(); // Show how long the read operation took.
    
    // Assuming Target is NULL (DO_IT_SMALL is defined), PDU.DATA contains the data read from PLC.
    // If Target pointed to 'Buffer', then 'Buffer' would contain the data.
    // These print statements are for debugging, showing the first few bytes received from the PLC.
    // Serial.print("First byte from PLC (PDU.DATA[0] or Buffer[0]): "); Serial.println(Target == NULL ? PDU.DATA[0] : Buffer[0]);
    // Serial.print("Second byte from PLC (PDU.DATA[1] or Buffer[1]): "); Serial.println(Target == NULL ? PDU.DATA[1] : Buffer[1]);
    
    // Extract data from the PLC response buffer (pointed to by Target, or PDU.DATA if Target is NULL).
    // The offsets (e.g., 14, 20, 24, 16) are byte offsets *within the data packet received from the PLC*.
    // These must correspond to the structure of DBNum in the PLC program where this data is stored.
    
    // S7.IntegerAt(buffer_pointer, byte_offset_in_buffer)
    MfrequanceI = S7.IntegerAt(Target, 14); // Read a 16-bit Integer from byte offset 14 of the received data.
                                           // This is likely the frequency command or status from the PLC.
    
    // S7.BitAt(buffer_pointer, byte_offset_in_buffer, bit_offset_in_byte)
    P60 = S7.BitAt(Target, 20, 0);         // Read Bit 0 from byte offset 20 of received data. (PLC control signal for 'marche' - e.g., run/stop command).
    Rege = S7.BitAt(Target, 24, 0);        // Read Bit 0 from byte offset 24 of received data. (PLC control signal for 'lampe').
    D3 = S7.BitAt(Target, 16, 0);          // Read Bit 0 from byte offset 16 of received data. (PLC control signal for output 'S3').
    D4 = S7.BitAt(Target, 16, 1);          // Read Bit 1 from byte offset 16 of received data. (PLC control signal for output 'S4').
    
    // Control ESP32 outputs based on data received from PLC.
    if(P60){ // If P60 bit from PLC is true (1).
      digitalWrite(marche, LOW);  // Turn 'marche' output LOW (likely initiates AC drive start/run).
    } else { // If P60 bit is false (0).
      digitalWrite(marche, HIGH); // Turn 'marche' output HIGH (likely stops the AC drive or removes run command).
    }
      
    if(Rege){ // If Rege bit from PLC is true (1).
      digitalWrite(lampe, LOW);   // Turn 'lampe' output LOW (indicator lamp ON, assuming LOW activates the lamp relay).
    } else { // If Rege bit is false (0).
      digitalWrite(lampe, HIGH);  // Turn 'lampe' output HIGH (indicator lamp OFF).
    }
      
    if(D3){ // If D3 bit from PLC is true (1).
      digitalWrite(S3, LOW);      // Set 'S3' output LOW (active state).
    } else { // If D3 bit is false (0).
      digitalWrite(S3, HIGH);     // Set 'S3' output HIGH (inactive state).
    }
      
    if(D4){ // If D4 bit from PLC is true (1).
      digitalWrite(S4, LOW);      // Set 'S4' output LOW (active state).
    } else { // If D4 bit is false (0).
      digitalWrite(S4, HIGH);     // Set 'S4' output HIGH (inactive state).
    }
    
    // Map the frequency value received from PLC (MfrequanceI) to a servo motor angle (0-180 degrees).
    // The PLC provides a value (e.g., in the range 15 to 50, could represent Hz, RPM percentage, or a scaled value).
    // This is then scaled to the servo's typical 0-180 degree operational range.
    // Adjust the input range (15, 50) if the PLC sends a different range for frequency.
    int servoAngle = map(MfrequanceI, 15, 50, 0, 180); // map(value, fromLow, fromHigh, toLow, toHigh)
    myservo.write(servoAngle); // Command the servo to move to the calculated angle.
  }
  else // If PLC read operation failed (Result code is not 0).
  {
    CheckError(Result); // Check the error code, print it, and disconnect if it's severe.
  }
    
  delay(500); // Wait for 500 milliseconds before starting the loop again.
              // This determines the cycle rate of PLC communication and I/O updates.
              // Adjust as needed for application responsiveness and network load.
}
