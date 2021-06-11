#include "Platform.h"
#include "Settimino.h"
#include <HCSR04.h>
#include <ESP32Servo.h>

#define DO_IT_SMALL

Servo myservo;  // create servo object to control a servo
HCSR04 hc(27,14); //initialisation class HCSR04 (trig pin , echo pin)

IPAddress Local(192,168,43,90); // Local Address
IPAddress PLC(192,168,43,52);   // PLC Address
char ssid[] = "Dar internen";    // Your network SSID (name)
char pass[] = "00000003";  // Your network password (if any)
IPAddress Gateway(192, 168,43, 1);
IPAddress Subnet(255, 255, 255, 0);


int DBNum = 2; // This DB must be present in your PLC
byte Buffer[1024];
S7Client Client;
unsigned long Elapsed;

int niveau; 
float MfrequanceF = 14;
int MfrequanceI = 15;
int servoPin = 15;
int times = 15000 ;
int frequenceval;
int LED_BUILTIN = 2;
const int marche = 26;
const int lampe = 25;
const int S3 = 33;
const int S4 = 32;
const int Ready = 21;
const int defaut = 19;
const int thermistors = 18;
const int inuse = 5;
int frequencepot = 34; 
int ADC_Max = 4096;
uint16_t count = 0;

void setup() {
Serial.begin(115200);
#ifdef S7WIFI
//--------------------------------------------- ESP8266 Initialization    
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    WiFi.config(Local, Gateway, Subnet);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.print("Local IP address : ");
    Serial.println(WiFi.localIP());
#else
//--------------------------------Wired Ethernet Shield Initialization    
    // Start the Ethernet Library
    EthernetInit(mac, Local);
    // Setup Time, someone said me to leave 2000 because some 
    // rubbish compatible boards are a bit deaf.
    delay(2000); 
    Serial.println("");
    Serial.println("Cable connected");  
    Serial.print("Local IP address : ");
    Serial.println(Ethernet.localIP());
#endif 
  myservo.setPeriodHertz(50); // Standard 50hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using SG90 servo min/max of 500us and 2400us
  // for MG995 large servo, use 1000us and 2000us,
  // which are the defaults, so this line could be
  digitalWrite(marche, HIGH); // "myservo.attach(servoPin);"
  digitalWrite(lampe, HIGH);
  digitalWrite(S3, HIGH);
  digitalWrite(32, HIGH);

  pinMode(marche, OUTPUT);
  pinMode(lampe, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Ready, INPUT_PULLUP);
  pinMode(defaut, INPUT_PULLUP);
  pinMode(thermistors, INPUT_PULLUP);
  pinMode(inuse, INPUT_PULLUP);
  
}
bool Connect()
{
    int Result=Client.ConnectTo(PLC, 
                                  0,  // Rack (see the doc.)
                                  2); // Slot (see the doc.)
    Serial.print("Connecting to ");Serial.println(PLC);  
    if (Result==0) 
    {
      
      Serial.print("Connected ! PDU Length = ");Serial.println(Client.GetPDULength());
      
    }
    else
      Serial.println("Connection error");
      
    return Result==0;
}
void CheckError(int ErrNo)
{
  Serial.print("Error No. 0x");
  Serial.println(ErrNo, HEX);
  
  // Checks if it's a Severe Error => we need to disconnect
  if (ErrNo & 0x00FF)
  {
    Serial.println("SEVERE ERROR, disconnecting.");
    Client.Disconnect(); 
  }
}
//----------------------------------------------------------------------
// Profiling routines
//----------------------------------------------------------------------
void MarkTime()
{
  Elapsed=millis();
}
//----------------------------------------------------------------------
void ShowTime()
{
  // Calcs the time
  Elapsed=millis()-Elapsed;
  Serial.print("Job time (ms) : ");
  Serial.println(Elapsed);   
}
void loop() {
  
  int ReadyVal = digitalRead(Ready);
  int defautVal = digitalRead(defaut);
  int thermistorsVal = digitalRead(thermistors);
  int inuseVal = digitalRead(inuse);
  int frequance ; 
  niveau = int(hc.dist()); 
  frequenceval = analogRead(frequencepot);
  Serial.println(frequenceval);
  frequenceval = map(frequenceval, 0, ADC_Max, 0, 50);
  int Size, Result;
  void *Target;
 bool P60 , Rege , D3, D4;
#ifdef DO_IT_SMALL
  Size=25;
  Target = NULL; // Uses the internal Buffer (PDU.DATA[])
#else
  Size=1024;
  Target = &Buffer; // Uses a larger buffer
#endif
  
  // Connection
  while (!Client.Connected)
  {
    if (!Connect())
      delay(500);
  }
  if (Connect())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
      }
      if (ReadyVal == HIGH) {
        
         
    S7.SetBitAt(0,0,0);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         0,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  } else {
    
    S7.SetBitAt(0,0,1);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         0,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  }
       if (thermistorsVal == HIGH) {
       
    S7.SetBitAt(0,0,0);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         8,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  } else {
    digitalWrite(marche, HIGH);        
    S7.SetBitAt(0,0,1);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         8,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
      S7.SetBitAt(0,0,0);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         20,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  }
        if (defautVal == HIGH) {
         
    S7.SetBitAt(0,0,0);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         12,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  } else {
         
    S7.SetBitAt(0,0,1);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         12,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  }
       if (inuseVal == HIGH) {
        
    S7.SetBitAt(0,0,0);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         4,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  } else {
         
    S7.SetBitAt(0,0,1);
    Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         4,        // Start from byte N.0
                         1,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  }

  S7.SetIntAt(0,niveau);
      Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         18,        // Start from byte N.0
                         2,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
 S7.SetIntAt(0,frequenceval);
      Client.WriteArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         6,        // Start from byte N.0
                         2,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  
  Serial.print("Reading ");Serial.print(Size);Serial.print(" bytes from DB");Serial.println(DBNum);
  // Get the current tick
  MarkTime();
  Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
                         DBNum,    // DB Number
                         0,        // Start from byte N.0
                         Size,     // We need "Size" bytes
                         Target);  // Put them into our target (Buffer or PDU)
  if (Result==0)
  {
    ShowTime();
    Serial.println(PDU.DATA[0]); // Print the first byte received
    Serial.println(PDU.DATA[1]); // Print the second… and so on
    MfrequanceI=S7.IntegerAt(14);
    P60 = S7.BitAt(20,0);
    Rege = S7.BitAt(24,0);
    D3 = S7.BitAt(16,0);
    D4 = S7.BitAt(16,1);
    

    if(P60){
      digitalWrite(marche, LOW);
      }
      else {
        digitalWrite(marche, HIGH);
        }
      if(Rege){
        digitalWrite(lampe, LOW);
      }
      else {
        digitalWrite(lampe, HIGH);
        }
      if(D3){
        digitalWrite(S3, LOW);
      }
      else {
       digitalWrite(S3, HIGH);
        }
      if(D4){
        digitalWrite(S4, LOW);;
      }
      else {
        digitalWrite(S4, HIGH);
        }
     MfrequanceI = map(MfrequanceI, 15, 50, 0, 180);
     myservo.write(MfrequanceI);          
    
  }
  else
    CheckError(Result);
    
  delay(500);
}
