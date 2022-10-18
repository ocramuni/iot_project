// Wi-Fi AC Power Meter 

// Libraries
#include <Statistical.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08DefaultTLSConfig.h>
#include <LiquidCrystal_PCF8574.h> 
#include <Wire.h>
#include <ArduinoMqttClient.h>
#include <WiFi101.h>
#include "arduino_secrets.h"

// Types

/** The calibration parameters structure type. */
typedef struct CALIBRATIONSTRUCT{
  bool  isValid;         ///< If true, calibration data are valid
  float voltageOffset1;  ///< Offset deviation and accuracy of voltage sensor
  float voltageOffset2;  ///< Offset value due to calculation error from square root of voltageData
  float currentOffset1;  ///< Offset deviation and accuracy of current sensor
  float currentOffset2;  ///< Offset value due to calculation error from square root of currentData
  float powerOffset;     ///< Offset deviation and accuracy of realPower
};

/** The calibration parameters union type. */
typedef union CALIBRATIONUNION{
  CALIBRATIONSTRUCT details;
  byte Data[32];
};

// Pin Mapping
const int currentAnalogInputPin = A0;  ///< Which pin to measure current Value
const int voltageAnalogInputPin = A1;  ///< Which pin to measure voltage Value
const int calibrationPin = 6;          ///< Button to perform calibration

// Global Constants
const int SLOT = 8;                     ///< ATECC508A EEPROM SLOT where to save calibration data
const int I2C_ADDR = 0x27;              ///< PCF8574 I2C adress
const int mVperAmpValue = 185;          ///< ACS712 current module : for 5A module key in 185, for 20A module key in 100, for 30A module key in 66
const unsigned long periodLCD = 5000;   ///< refresh every X seconds (in seconds) LCD Display. Default 5000 = 5 second 
const int decimalPrecision = 1;         // decimal places for current (x2), voltage, wattage and apparent power shown in LED Display & Serial Monitor
const float outputGain = 1.5;           // amplification factor of voltage sensor output (to overcome wave limit cut near 250Vac)

// Wi-Fi
const char ssid[] = SECRET_SSID;        ///< network SSID
const char pass[] = SECRET_PASS;        ///< Wi-Fi network password

// MQTT
const char brokerAddress[] = "fedtop.home.arpa";        ///< MQTT Broker Hostname
const int brokerPort = 1883;                            ///< MQTT Broker Port
const char brokerUsername[] = SECRET_MQTT_SERVER_USERNAME;
const char brokerPassword[] = SECRET_MQTT_SERVER_PASSWORD;
const char clientPrefix[] = "pwrmtr";
const char sensorsPrefix[] = "sensors/";
const char powerPrefix[] = "power/";

// Global Variables

/**
 * MQTT
 */
String clientId = String(clientPrefix);
String voltageTopic = String(sensorsPrefix);
String currentTopic = String(sensorsPrefix);
String realPowerTopic = String(powerPrefix);
String apparentPowerTopic = String(powerPrefix);

/**
 * Wi-Fi
 */
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
IPAddress ipAddress;  // IP address of Wi-Fi card

/**
 * Calibration
 */
CALIBRATIONUNION calibrationInfo;
int saveCalibrationData = 0;        ///< Save calibration data to EEPROM
int makeVoltageCalibration = 0;     ///< Start AC Voltage calibration process
int makeCurrentCalibration = 0;     ///< Start AC Current calibration process
int makePowerCalibration = 0;       ///< Start AC Power calibration process

/**
 * AC Voltage Measurement
 */
float voltageData[1000];          ///< accumulation of voltage sample readings
Array_Stats<float> voltageDataArray(voltageData, sizeof(voltageData) / sizeof(voltageData[0]));

float voltageSampleRead  = 0;     // to read the value of a voltage sample
float voltageLastSample  = 0;     // to count time for each voltage sample (~ 1 milli second)
int voltageSampleCount = 0;     // to count number of voltage sample
float RMSVoltageMean;             ///< RMS value of voltageData

float voltageOffset1 = 0;                   // to offset deviation and accuracy. Offset any fake voltage when no voltage operates
float voltageOffset2 = 0;                   // to offset value due to calculation error from RMS of voltageData
float voltageSampleSumOffset =0;            // accumulation of sample readings for voltage offset
float offsetVoltageMean = 0;                // to calculate the average value from all voltage samples for offset
float voltageOffsetLastSample = 0;          // to count time for each voltage sample for offset purpose
float voltageOffsetSampleCount = 0;         // to count number of voltage sample for offset

/**
 * AC Current Measurement
 */
float currentData[1000];          ///< accumulation of current sample readings
Array_Stats<float> currentDataArray(currentData, sizeof(currentData) / sizeof(currentData[0]));

float currentSampleRead = 0;                // to read the value of a current sample
float currentLastSample = 0;                // to count time for each current sample (~ 1 milli second)
int currentSampleCount = 0;               // to count number of current sample
float RMSCurrentMean = 0;                   // RMS of currentData
float FinalRMSCurrent;                      // the final RMS current reading (after ACS712 voltage conversion)

float currentOffset1 = 0;                   // to Offset deviation and accuracy. Offset any fake current when no current operates.  
float currentOffset2 = 0;                   // to offset value due to calculation error from RMS of currentData.
float currentSampleSumOffset = 0;           // accumulation of sample readings for current offset
float offsetCurrentMean = 0;                // to calculate the average value from all current samples for offset
float currentOffsetLastSample = 0;          // to count time for each current sample for offset purpose
float currentOffsetSampleCount = 0;         // to count number of current sample for offset

/**
 * AC Power Measurement
 */
float sampleCurrent1;                       // to read the value of a current sample
float sampleCurrent2;                       // to convert currentAnalogInputPin read voltage in 0-5000 milli volts
float sampleCurrent3;                       // use to calculate current final value
float apparentPower;                        ///< apparent power reading (VA)
float realPower = 0;                        ///< real power reading (W)
float powerSampleRead = 0;                  // to read the current real power sample value
float powerLastSample = 0;                  // to count time for each real power sample (~ 1 milli second)
float powerSampleCount = 0;                 // to count number of real power sample
float powerSampleSum = 0;                   // accumulation of real power sample readings

float powerOffset = 0;                      // to offset value due to calculation error from realPower
float powerOffsetLastSample = 0;            // to count time for each real power sample for offset purpose
float powerOffsetSampleCount = 0;           // to count number of real power sample for offset

/**
 * LCD Display
 */
LiquidCrystal_PCF8574 LCD(I2C_ADDR);        // set the LCD address to I2C_ADDR
unsigned long startMillisLCD;               // start counting time for LCD Display
unsigned long currentMillisLCD;             // current counting time for LCD Display
int page = 1;                               // flip page to display values, if 0 show calibration message


void setup() {                                      
  // Initialize serial bus (Serial Monitor) 
  Serial.begin(9600);  // to display readings in Serial Monitor at 9600 baud rates

  // Get calibration data from ATECC508A device
  configureECCX08();
  
  if(calibrationInfo.details.isValid) {
    // Use calibration data read from ATECC508A EEPROM
    Serial.println("Calibration data are valid");
    voltageOffset1 = calibrationInfo.details.voltageOffset1;
    voltageOffset2 = calibrationInfo.details.voltageOffset2;
    currentOffset1 = calibrationInfo.details.currentOffset1;
    currentOffset2 = calibrationInfo.details.currentOffset2;
    powerOffset = calibrationInfo.details.powerOffset;
  } 

  // Configure Display LCD
  configureLCD();

  // Connect to Wi-Fi network
  configureWIFI();

  // Connect to MQTT broker
  configureMQTT();
  
  startMillisLCD = millis();  // Start counting time for LCD display
  
  // Pin configuration
  pinMode(calibrationPin, INPUT_PULLUP);  // utilize microprocessor's internal pull-up resistor
  delay(1);                               // wait to be sure that calibrationPin is LOW
}


void loop() {                                      

  /*
   * Calibration
   * Offset will be automatically calibrate on first boot after programming
   * or when calibrationPin is connected to GND.  
   */
  if (digitalRead(calibrationPin) == LOW or !calibrationInfo.details.isValid) {
    makeVoltageCalibration = 1;
    makeCurrentCalibration = 1;
    makePowerCalibration = 1;
    page = 0;
    LCD.clear();  // clear display from old values
    Serial.println("Starting calibration");
  }

  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  /*
   * AC Voltage Measurement
   * Read voltage value from ZMPT101B AC Voltage Sensor
   */
  if(millis() >= voltageLastSample + 1 ) {                                                        // take 1 read every 1 milli second
    voltageSampleRead = outputGain * (analogRead(voltageAnalogInputPin)- 512) + voltageOffset1;   // read the voltage sample value
    voltageSampleSumOffset = voltageSampleSumOffset + voltageSampleRead;                          // values accumulate for offset purpose every milli second
    voltageSampleCount = voltageSampleCount + 1;
    voltageData[voltageSampleCount - 1] = voltageSampleRead;                                      // accumulate voltage sample readings in an array
    voltageLastSample = millis();
  }
  // display voltage value on serial monitor
  if(voltageSampleCount == 1000)                                                              // after 1000 samples (~ 1 second), do the calculation and display value
    {
      offsetVoltageMean = voltageSampleSumOffset / voltageSampleCount;                        // average the offset reading
      RMSVoltageMean = voltageDataArray.Average(voltageDataArray.RMS_Avg) + voltageOffset2;   // get RMS of voltage samples
      if (Serial) {
        Serial.print(RMSVoltageMean, decimalPrecision);                                         // display RMS Voltage on serial terminal
        Serial.print(" V   ");
      }
      if (makeVoltageCalibration == 0) {
        // send message to MQTT
        mqttClient.beginMessage(voltageTopic);
        mqttClient.print(RMSVoltageMean, decimalPrecision);
        mqttClient.endMessage();
      }
      voltageSampleCount = 0;
      voltageSampleSumOffset = 0;
      memset(voltageData, 0, sizeof(voltageData));                                            // reset voltage accumulation array
    }

  /*
   * Offset AC Voltage
   */
  // Calculate offset from voltage samples mean
  if(makeVoltageCalibration == 1) {                                                   // Run this code when calibration is required
      voltageOffset1 = 0; 
      if(millis() >= voltageOffsetLastSample + 1) {                                   // keep countng time for offset1
        voltageOffsetSampleCount = voltageOffsetSampleCount + 1;                      // every 1 milli second add 1 count
        voltageOffsetLastSample = millis();
      }                                             
      if(voltageOffsetSampleCount == 2000) {                                          // after 2 seconds, run this codes                                        
        voltageOffset1 = -offsetVoltageMean;                                          // set the offset values (using absolute value)
        makeVoltageCalibration = 2;                                                   // go for second offset Settings
        voltageOffsetSampleCount = 0;                                          
      } 
    }   
  // Calculate offset value from RMS of voltageData
  if(makeVoltageCalibration == 2) {                                                   // Run this code after first offset done
      voltageOffset2 = 0;                                                             
      if(millis() >= voltageOffsetLastSample + 1) {                                   // keep countng time for offset2                                                                       
        voltageOffsetSampleCount = voltageOffsetSampleCount + 1;                                                                          
        voltageOffsetLastSample = millis();                                                                          
      }                                                                                
      if(voltageOffsetSampleCount == 2000) {                                          // after 2 seconds, run this codes
          voltageOffset2 = -RMSVoltageMean;                                           // set the offset values (using absolute value)
          makeVoltageCalibration = 0;                                                 // change the offset mode to original
          voltageOffsetSampleCount = 0;
        }                                                                             
    } 

  /*
   * AC Current Measurement
   * Read current value from ACS712 Current Sensor
   */
  if(millis() >= currentLastSample + 1) {                                                     // take 1 read every 1 milli second
    currentSampleRead = analogRead(currentAnalogInputPin) - 512 + currentOffset1;             // read the current sample value
    currentSampleSumOffset = currentSampleSumOffset + currentSampleRead;                      // accumulate offset value
    currentSampleCount = currentSampleCount + 1;
    currentData[currentSampleCount - 1] = currentSampleRead;                                  // accumulate voltage sample readings in an array
    currentLastSample = millis();
  }
  // display current value on serial monitor
  if(currentSampleCount == 1000) {                                                            // after 1000 samples (~ 1 second), do the calculation and display value
      offsetCurrentMean = currentSampleSumOffset / currentSampleCount;                        // average the offset reading
      RMSCurrentMean = currentDataArray.Average(currentDataArray.RMS_Avg) + currentOffset2;   // get RMS of current samples
      //((RMSCurrentMean / 1024) * 5000)) is converting the read voltage in 0-5000 milli volts
      FinalRMSCurrent = (((RMSCurrentMean / 1024) * 5000) / mVperAmpValue);                   // calculate the final RMS current
      if (Serial) {
        Serial.print(FinalRMSCurrent, decimalPrecision*2);                                      // display RMS Voltage on serial terminal
        Serial.print(" A   ");
      }
      if (makeCurrentCalibration == 0) {
        mqttClient.beginMessage(currentTopic);
        mqttClient.print(FinalRMSCurrent, decimalPrecision*2);
        mqttClient.endMessage();
      }
      currentSampleCount = 0;
      currentSampleSumOffset = 0;
      memset(currentData, 0, sizeof(currentData));                                            // reset current accumulation array
    }

  /*
   * Offset AC Current
   */
  // Calculate offset from current samples mean
  if(makeCurrentCalibration == 1) {                                                           // Run this code when calibration is required
    currentOffset1 = 0;
    if(millis() >= currentOffsetLastSample + 1) {                                             // keep countng time for offset1*/                                 
      currentOffsetSampleCount = currentOffsetSampleCount + 1;                                                                          
      currentOffsetLastSample = millis();                                                                          
    }                                                                                 
    if(currentOffsetSampleCount == 2000) {                                                    // after 2 seconds, run this codes
      currentOffset1 = -offsetCurrentMean;                                                    // set the offset values (using absolute value)
      makeCurrentCalibration = 2;                                                             // go for second offset Settings
      currentOffsetSampleCount = 0;                                              
    } 
  }   
  // Get offset value from RMS of currentData
  if(makeCurrentCalibration == 2) {                                                           // Run this code after first offset done
    currentOffset2 = 0;
    if(millis() >= currentOffsetLastSample + 1) {                                             // keep counting time for offset2
      currentOffsetSampleCount = currentOffsetSampleCount + 1;
      currentOffsetLastSample = millis();
    }                                                                             
    if(currentOffsetSampleCount == 2000) {                                                    // after 2 seconds, run this codes
      currentOffset2 = -RMSCurrentMean;                                                       // set the offset values (using absolute value)
      makeCurrentCalibration = 0;                                                             // change the offset mode to original
      currentOffsetSampleCount = 0;
    }                                                                             
  } 

 /*
  * AC Power Measurement
  * Read current ans voltage values to calculate real power.
  * Use RMS current and RMS voltage to calculate apparent power.
  */
  if(millis() >= powerLastSample + 1) {                                                             // take 1 reading every 1 milli second
    sampleCurrent1 = analogRead(currentAnalogInputPin) - 512 + currentOffset1;                      // read the current sample value
    sampleCurrent2 = (sampleCurrent1 / 1024) * 5000;                                                // converting the read voltage in 0-5000 milli volts
    sampleCurrent3 = sampleCurrent2 / mVperAmpValue;                                                // calculate final current value
    voltageSampleRead = outputGain * (analogRead(voltageAnalogInputPin) - 512) + voltageOffset1 ;   // read the voltage sample value
    powerSampleRead = voltageSampleRead * sampleCurrent3 ;                                          // real power sample value
    powerSampleSum = powerSampleSum + powerSampleRead ;                                             // accumulate power value with older sample readings
    powerSampleCount = powerSampleCount + 1;
    powerLastSample = millis();
  }
  // display power values on serial monitor
  if(powerSampleCount == 1000) {                                                              // after 1000 samples (~ 1 second), do the calculation and display value
    realPower = ((powerSampleSum / powerSampleCount) + powerOffset) ;                         // calculate average value of all power sample readings
    if (Serial) {
      Serial.print(realPower, decimalPrecision);                                                // display real power on serial terminal
      Serial.print(" W   ");
    }
    if (makePowerCalibration == 0) {
      mqttClient.beginMessage(realPowerTopic);
      mqttClient.print(realPower, decimalPrecision);
      mqttClient.endMessage();
    }
    apparentPower = FinalRMSCurrent * RMSVoltageMean;                                         // Apparent power do not need to recount as RMS current and RMS voltage values available
    if (Serial) {
      Serial.print(apparentPower, decimalPrecision);                                            // display apparent power on serial terminal
      Serial.println(" VA ");
    }
    if (makePowerCalibration == 0) {
      mqttClient.beginMessage(apparentPowerTopic);
      mqttClient.print(apparentPower, decimalPrecision);
      mqttClient.endMessage();
    }
    powerSampleSum = 0;
    powerSampleCount = 0;
  }

 /*
   * Offset AC Power
   */
  // Calculate offset using real power value
  if(makePowerCalibration == 1) {                                     // Run this code when calibration is required
    powerOffset = 0;                            
    if(millis() >= powerOffsetLastSample + 1) {
      powerOffsetSampleCount = powerOffsetSampleCount + 1;
      powerOffsetLastSample = millis();
    } 
    if(powerOffsetSampleCount == 5000) {                              // after 5 seconds, run this codes.
      powerOffset = -realPower;                                       // set the offset values (using absolute value)
      makePowerCalibration = 0;                                       // change the offset mode to original
      powerOffsetSampleCount = 0;
      saveCalibrationData = 1;                                        // now we have all calibration data, save them to EEPROM
    }
  }

  // save all calibration data to ATECC508A EEPROM
  if (saveCalibrationData == 1) {
    calibrationInfo.details.isValid = true;
    calibrationInfo.details.voltageOffset1 = voltageOffset1;
    calibrationInfo.details.voltageOffset2 = voltageOffset2;
    calibrationInfo.details.currentOffset1 = currentOffset1;
    calibrationInfo.details.currentOffset2 = currentOffset2;
    calibrationInfo.details.powerOffset = powerOffset;

    if (!ECCX08.writeSlot(SLOT, calibrationInfo.Data, sizeof(calibrationInfo.Data))) {
      if (Serial)
        Serial.println("Writing ECCX08 data failed!");
      while (1);
    } else {
      if (Serial)
        Serial.println("ECCX08 data uploaded successfully");
      LCD.begin(16, 2); // re-initialize the LCD (https://github.com/arduino-libraries/ArduinoECCX08/issues/22)
      saveCalibrationData = 0;
      page = 1;
    }  
  }

  /*
   * LCD Display
   */
  currentMillisLCD = millis();
  if (currentMillisLCD - startMillisLCD >= periodLCD) {    // every periodLCD seconds
    if(page == 0) {                                        // We are waiting to get calibration data, do not display values
      LCD.setCursor(0,0);                                  // Set cursor to first colum 0 and first row 0
      LCD.print("Calibration");
      LCD.setCursor(0,1);                                  // Set cursor to first colum 0 and second row 1
      LCD.print("in progress...");
    } else if(page == 1) {
      LCD.clear();
      LCD.setCursor(0, 0); // first line
      LCD.print(clientId);
      LCD.setCursor(0, 1); // second line
      LCD.print(ipAddress);
      page = 2;
    } else {
      LCD.clear();
      LCD.setCursor(0,0);                                  // Set cursor to first colum 0 and first row 0
      LCD.print(FinalRMSCurrent, decimalPrecision * 2);       // display current value in LCD in first row
      LCD.print("A    ");
      LCD.setCursor(8,0);
      LCD.print(RMSVoltageMean, decimalPrecision);          // display voltage value in LCD in first row
      LCD.print("V     ");
      LCD.setCursor(0,1);                                  // Set cursor to first colum 0 and second row 1
      LCD.print(realPower, decimalPrecision);               // display real power value in LCD in second row
      LCD.print("W     ");
      LCD.setCursor(8,1);
      LCD.print(apparentPower, decimalPrecision);           // display apparent power value in LCD in second row
      LCD.print("VA    ");
      page = 1;
    }
    startMillisLCD = currentMillisLCD ;
  }
  
}

/**
 * Configures and initializes ATECC508A CryptoAuthentication Device.
 *
 * Use EEPROM on device to store calibration data.
 */
void configureECCX08() {
  // Configure the device
  if (!ECCX08.begin()) {
    if (Serial)
      Serial.println("No ECCX08 present!");
    while (1);
  }

  String serialNumber = ECCX08.serialNumber();

  if (Serial) {
    Serial.print("ECCX08 Serial Number = ");
    Serial.println(serialNumber);
  }

  if (!ECCX08.locked()) {
    if (!ECCX08.writeConfiguration(ECCX08_DEFAULT_TLS_CONFIG)) {
      if (Serial)
        Serial.println("Writing ECCX08 configuration failed!");
      while (1);
    }

    if (!ECCX08.lock()) {
      if (Serial)
        Serial.println("Locking ECCX08 configuration failed!");
      while (1);
    }

    if (Serial) {  
      Serial.println("ECCX08 locked successfully");
      Serial.println();
    }
  }

  // Read ECCX08 EEPROM
  if (!ECCX08.readSlot(SLOT, calibrationInfo.Data, sizeof(calibrationInfo.Data))) {
    if (Serial)
      Serial.println("Reading ECCX08 data failed!");
    while (1);
  } else {
    if (Serial)
      Serial.println("ECCX08 data read successfully");
  }
}

/**
 * Configures and initializes PCF8574 LCD Display.
 */
void configureLCD() {
  int error;
  Wire.begin(); // Test for a I2C device.
  Wire.beginTransmission(I2C_ADDR);
  error = Wire.endTransmission();

  if (error == 0) {
    if (Serial)
      Serial.println("LCD found.");
    LCD.begin(16, 2); // initialize the LCD for a 16 chars and 2 line display
    LCD.setBacklight(128);
    LCD.home();
    LCD.clear();
  } else {
    if (Serial)
      Serial.println("LCD not found.");
  }
}


/**
 * Connect to Wifi network
 */
void configureWIFI() {
  if (Serial) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
  }
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    if (Serial)
      Serial.print(".");
    delay(5000);
  }

  ipAddress = WiFi.localIP();
  if (Serial) {
    Serial.println("You're connected to the network");
    Serial.print("Your ip address is: ");   // print the local IP address
    Serial.println(ipAddress);
  }  
}


/**
 * Connect to MQTT brocker.
 */
void configureMQTT() {
  if (Serial) {
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(brokerAddress);
  }

  // pad octets of IP Address with Zeros
  String ipAddress2 = String(ipAddress[2]);
  while (ipAddress2.length() != 3) {
    ipAddress2 = '0' + ipAddress2;
  }
  clientId += ipAddress2;
  String ipAddress3 = String(ipAddress[3]);
  while (ipAddress3.length() != 3) {
    ipAddress3 = '0' + ipAddress3;
  }
  clientId += ipAddress3;    // Each client must have a unique client ID
  mqttClient.setId(clientId);

  mqttClient.setUsernamePassword(brokerUsername, brokerPassword);

  if (!mqttClient.connect(brokerAddress, brokerPort)) {
    if (Serial) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
    }
    while (1);
  }

  if (Serial)
    Serial.println("You're connected to the MQTT broker!");

  voltageTopic += String(clientId + "/voltage");
  currentTopic += String(clientId + "/current");
  realPowerTopic += String(clientId + "/real");
  apparentPowerTopic += String(clientId + "/apparent");
}
