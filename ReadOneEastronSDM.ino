/*!
  Program for Arduino Mega 2560 to read SDM-energy-meters
   - with internet HAT
   - RTC with battery and temperature censor, I2C (SCL/SDA) connected
   - with LCD board, I2C (SCL/SDA) connected
   - with RS485 (FTD to RS485) connected to RX-TX
   - main goal to read values from 4 SDM-modbus energy monitors
   - Export to MQTT (to HomeDomotics - HomeAssistant)

  << version-update-text removed>> 
  << all code for RTC removed >>
  << all code for LCD removed, including most SerialPrint (to LCD) >>
  << some SerialPrints (to LCD) not removed because it is explaining comment >>
  << all code for Internet(HAT) removed >>
  << all code for SDM 2 - 4 removed: only one SDM readout >>
  << all code for exporting values to Domotica via MQTT removed >>
  So this is not code to be ready for compilation, only to show the essential codelines
   
  10-jun-2020 - PROG057 
   * original code from http://www.bizkit.ru/en/2019/02/21/12563/
   * updated ESP32 -> WemosD1mini with ESP8266 = other pin layout
   * first testing with SDM630 with Slave_ID = 2
   * '&amp;' -> '&'

   * Connections: 
   *   Mega-RX2 = pin17 -> RS485-DO
   *   Mega-TX2 = pin16 -> RS485-DI
   *   Mega-pin19       -> RS485-DE & RS485-RE
   * Uncomment MAX485_DE because hardware connected to RE_NEG
   * Calculate result value correctly by combining two registers


*/

#include "arduino_secrets.h"
#include "ModbusMaster.h" //https://github.com/4-20ma/ModbusMaster
#include <SPI.h>      // Comm with internet hat

// RS485 definitions
// RS485 has a enable/disable pin to transmit or receive data. Using Arduino Rx/Tx 'Enable'; High to Transmit, Low to Receive
// Serial2 uses RX-PIN 17 = RX2
//              TX_PIN 16 = TX2
#define MAX485_RE_NEG 19      // RE = Receive Enable (default negative). DE = Driver Enable is connected to same pin
#define Slave_ID1      1      // Each slave (device on the modbus) has its own ID
#define ModbusBaud  9600      // speed of the modbus
#define FREQ_SDMMillis 5000   // update every 1000ms = 1 sec
long lastSDMMillis = 0;
 
// instantiate ModbusMaster object
ModbusMaster modbus1;

// Routines needed for RS485-communication
void preTransmission()
{
  // Switch to transmit data,
  digitalWrite(MAX485_RE_NEG, HIGH);  // Receive Enable OFF = Negative HIGH. Driver Enable is ON.
  //  Serial.println("---preTransmission");
}

void postTransmission()
{
  // Switch to receive data
  digitalWrite(MAX485_RE_NEG, LOW);   // Receive Enable ON = Negative LOW. Driver Enable is OFF.
  //  Serial.println("--postTransmission");
}

/*******************************************************************************************************\
 *                         SETUP                                                                       *
\*******************************************************************************************************/
void setup()
{

  // Init RS-485 in receive mode
  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);
  Serial.println("- RE pin RS485 initialized");

  // Modbus communication runs at 9600 baud and ArduinoID-terminal at 115200 baud
  Serial2.begin(ModbusBaud, SERIAL_8N1);
  modbus1.begin(Slave_ID1, Serial2);    
  Serial.println("- RS485 serial line set up");

  // Callbacks allow us to configure the RS485 transceiver correctly
  modbus1.preTransmission(preTransmission);
  modbus1.postTransmission(postTransmission);
  Serial.println("- pre and postTransmission functions set up");

}

/*******************************************************************************************************\
 *                         LOOP                                                                        *
\*******************************************************************************************************/
void loop() 
{
  
  // Connect to SDM by RS-485
  long currentSDMMillis = millis();
  if (currentSDMMillis - lastSDMMillis > FREQ_SDMMillis)                    // poll every 5 seconds
  {
    Serial.println("====== SDM via RS-485 =====START==");
    LeesSDM1Volt();
    LeesSDM1Total();

    lastSDMMillis = currentSDMMillis;
    Serial.println("====== SDM via RS-485 =====END====");
  }
}

/*******************************************************************************************************\
 *                         RESULT MSG MODBUS                                                           *
\*******************************************************************************************************/
bool getResultMsg(ModbusMaster *node, uint8_t result) {
  String ResultString = ""; 
  
  switch (result) 
  {
  case node->ku8MBSuccess:
    ResultString += "..Success..";
    return true;
    break;
  case node->ku8MBIllegalFunction:
    ResultString += "Illegal Function";
    break;
  case node->ku8MBIllegalDataAddress:
    ResultString += "Illegal Data Address";
    break;
  case node->ku8MBIllegalDataValue:
    ResultString += "Illegal Data Value";
    break;
  case node->ku8MBSlaveDeviceFailure:
    ResultString += "Slave Device Failure";
    break;
  case node->ku8MBInvalidSlaveID:
    ResultString += "Invalid Slave ID";
    break;
  case node->ku8MBInvalidFunction:
    ResultString += "Invalid Function";
    break;
  case node->ku8MBResponseTimedOut:
    ResultString += "Response Timed Out ";
    break;
  case node->ku8MBInvalidCRC:
    ResultString += "Invalid CRC";
    break;
  default:
    ResultString += "Unknown error: " + String(result);
    break;
  }
  Serial.print(" = ");
  Serial.println(ResultString);
  return false;
}


/*******************************************************************************************************\
 *                         READ REGISTERS                                                              *
\*******************************************************************************************************/
void LeesSDM1Volt() {
    // read registers starting from 30000
    int aantal_registers = 14;
    uint8_t result = modbus1.readInputRegisters(0x30000, aantal_registers);   // from register en so many values/registers 
    unsigned long l;  // results
    Serial.print("- Registers lezen SDM-1 vanaf 0x30000");

    if (getResultMsg(&modbus1, result)) {                 // handle result, also when error response
      // initiate enough room
      uint16_t data[aantal_registers];

      // get registers in sets of two
      for (int j=0;j<aantal_registers;j=j+2) {
        data[j]   = modbus1.getResponseBuffer(j);          // get data from buffer
        data[j+1] = modbus1.getResponseBuffer(j+1);
        l = data[j] * 65536 + data[j+1];  // convert data to one long

        Serial.println();
        Serial.print("Value [");
        Serial.print(30000+j);
        Serial.print("] = ");
        Serial.print(l);
        
        switch(j) {
          case 0:
            PublishValue(MQ_topic_id1_volt, l);	// received & calculated value in l is voltage - send to MQTT
            break;
          case 6:
            PublishValue(MQ_topic_id1_amp, l);  // received & calculated value in l is ampere - send to MQTT
            break;
          case 12:
            PublishValue(MQ_topic_id1_pwr, l);  // received & calculated value in l is power - send to MQTT
            break;
          default:
            // do nothing
            break;
        }
      }
      
    } else {
      PublishError("Error reading SDM1-0x30000-registers");
    }
}  

void LeesSDM1Total() {
    // read registers starting from 0x30156
    int aantal_registers = 2;
    uint8_t result = modbus1.readInputRegisters(0x30156, aantal_registers);   // from register en so many values/registers 
    unsigned long l;  // results
    Serial.print("- Registers lezen SDM-1 vanaf 0x30156");
    
    if (getResultMsg(&modbus1, result)) {                 // handle result, also when error response
      // initiate enough room
      uint16_t data[aantal_registers];

      // get registers in sets of two
      for (int j=0;j<aantal_registers;j=j+2) {
        data[j]   = modbus1.getResponseBuffer(j);          // get data from buffer
        data[j+1] = modbus1.getResponseBuffer(j+1);
        l = data[j] * 65536 + data[j+1];  // convert data to one long

        Serial.println();
        Serial.print("Value [");
        Serial.print(30156+j);
        Serial.print("] = ");
        Serial.print(l);
        
        switch(j) {
          case 0:
            PublishValue(MQ_topic_id1_enyT, l);
            break;
          default:
            // do nothing
            break;
        }
      }

    } else {
      PublishError("Error reading SDM1-0x30156-registers");
    }
}  

