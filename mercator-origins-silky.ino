// Mark B. Jones - Scuba Hacker! - 20 May 2023 - MIT Licence
//
// Silky is the audio module for Mercator Origins
//
// Integration info for the Adafruit SPI Flash SD Card - XTSD 512MB with the Beetle ESP32-C3 dev board
// https://www.dfrobot.com/product-2566.html
// https://learn.adafruit.com/adafruit-spi-flash-sd-card
// https://randomnerdtutorials.com/esp32-microsd-card-arduino/
//
// Time of flight sensor:
// https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cx.html

// Tutorial for getting files onto the Flash/SD by uploading to esp32:
//    https://community.appinventor.mit.edu/t/esp32-wifi-webserver-upload-file-from-app-to-esp32-sdcard-reader-littlefs/28126/3


#include "FS.h"
#include "SD.h"
#include "SPI.h"

//#include "SD-card-API.h"

#define USB_SERIAL Serial

#define ENABLE_DBG
// #define ENABLE_PARSE_SERIAL

#include "DFRobot_MAX98357A_mercator.h"

#include "mercator_secrets.c"

bool writeLogToSerial = false;

/* // not currently used
// I2C Light Sensor
// see https://github.com/Starmbi/hp_BH1750
#include <hp_BH1750.h>
hp_BH1750 BH1750;
*/

/* // not currently used
// I2C Time of Flight sensor. Adafruit VL53L4CX
// see https://learn.adafruit.com/adafruit-vl53l4cx-time-of-flight-distance-sensor
#include <vl53l4cx_class.h>
#define DEV_I2C Wire
#define XSHUT_PIN A1
#define VL53L4CX_DEFAULT_DEVICE_ADDRESS 0x12

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, XSHUT_PIN);
*/
//#define ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
#define ENABLE_ESPNOW_AT_COMPILE_TIME

const bool enableOTAServer = false;          // over the air updates

const bool enableESPNow = !enableOTAServer; // cannot have OTA server on regular wifi and espnow concurrently running
const bool enableLightSensor = false;
const bool enableTimeOfFlightSensor = false;

bool lightSensorAvailable = false;
bool timeOfFlightSensorAvailable = false;

#ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
  // see mercator_secrets.c for wifi login globals
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  bool otaActiveListening=true;   // OTA updates toggle
  AsyncWebServer asyncWebServer(80);
#endif

#ifdef ENABLE_ESPNOW_AT_COMPILE_TIME
  #include <esp_now.h>
  #include <WiFi.h>

// ************** ESPNow variables **************
  uint16_t ESPNowMessagesDelivered = 0;
  uint16_t ESPNowMessagesFailedToDeliver = 0;
  
  const uint8_t ESPNOW_CHANNEL=1;
  const uint8_t ESPNOW_NO_PEER_CHANNEL_FLAG = 0xFF;
  const uint8_t ESPNOW_PRINTSCANRESULTS = 0;
  const uint8_t ESPNOW_DELETEBEFOREPAIR = 0;
  
  esp_now_peer_info_t ESPNow_mako_peer;
  bool isPairedWithMako = false;
  
  const int RESET_ESPNOW_SEND_RESULT = 0xFF;
  esp_err_t ESPNowSendResult=(esp_err_t)RESET_ESPNOW_SEND_RESULT;

  bool ESPNowActive = false;
#endif

const int beetleLed = 10;
const uint8_t I2S_AMP_BCLK_PIN = GPIO_NUM_0;      // YELLOW --> Audio AMP Pin was 0
const uint8_t I2S_AMP_LRCLK_PIN = GPIO_NUM_1;     // BLUE --> Audio AMP Pin 1
const uint8_t I2S_AMP_DIN_PIN = GPIO_NUM_2;       // GREEN --> Audio AMP Pin was 2

const uint8_t SD_CS_PIN = GPIO_NUM_7;

uint8_t cardType = CARD_SD; // was 0

const float defaultVolume = 5.0;
const float maxVolume = 5.0;
const float minVolume = 3.0;
float volume = defaultVolume;

const uint8_t numberOfTracks = 100;
const uint8_t defaultTrack  = 0;
uint8_t currentTrack  = defaultTrack;
const char* currentTrackFilename = NULL;

uint32_t lastAudioGuidancePlayedAt = 0;
const uint32_t audioGuidanceMinimumGap = 50;


String musicList[numberOfTracks];   // SD card music playlist

DFRobot_MAX98357A amplifier;   

void setup()
{
  pinMode(beetleLed,OUTPUT);

  // LED flash - we're alive!
  USB_SERIAL.begin(115200);
  int warmUp=10;
  
  while (warmUp--)
  {
    digitalWrite(beetleLed,HIGH);
    delay(250);
    digitalWrite(beetleLed,LOW);
    delay(250);

    if (writeLogToSerial)
      USB_SERIAL.println("Warming up...");
  }

  // leave LED on
  digitalWrite(beetleLed,HIGH);

  if (writeLogToSerial)
     USB_SERIAL.println("\nHere we go...");

  delay(500);

/*
  if (enableLightSensor || enableTimeOfFlightSensor) 
  {
    DEV_I2C.begin();
  }

  delay(500);

  // light sensor and time of flight sensor are disabled
  if (enableLightSensor)
  {
    lightSensorAvailable = BH1750.begin(BH1750_TO_GROUND);// init the sensor with address pin connetcted to ground
                                              // result (bool) wil be be "false" if no sensor found
    if (!lightSensorAvailable) 
    {
       if (writeLogToSerial)
        USB_SERIAL.println("No BH1750 sensor found!");
    }
    else
    {
      if (writeLogToSerial)
       USB_SERIAL.println("BH1750 sensor initialised ok");
    }
  }

  if (enableTimeOfFlightSensor)
  {
    // Configure VL53L4CX
    sensor_vl53l4cx_sat.begin();
  
    // Switch off VL53L4CX
    sensor_vl53l4cx_sat.VL53L4CX_Off();

    //Initialize VL53L4CX
    VL53L4CX_Error initError = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);

    if (initError == VL53L4CX_ERROR_NONE)
    {
      if (writeLogToSerial)
        USB_SERIAL.println("Adafruit VL53L4CX Time Of Flight Sensor Initialised ok");

      VL53L4CX_Error measureError = sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();  

      if (measureError == VL53L4CX_ERROR_NONE)
      {
        if (writeLogToSerial)
           USB_SERIAL.println("Adafruit VL53L4CX Time Of Flight Sensor Started Measurement ok");
        timeOfFlightSensorAvailable = true;
      }
      else
      {
        const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(measureError);
        if (writeLogToSerial)
           USB_SERIAL.printf("\nError: Adafruit VL53L4CX Time Of Flight Sensor startup measurement error: %i %s\n",measureError, TOFErrorBuffer);
        timeOfFlightSensorAvailable = false;
      }
    }
    else
    {
      const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(initError);
      if (writeLogToSerial)
         USB_SERIAL.printf("Error: Adafruit VL53L4CX Time Of Flight Sensor initialisation error: %i %s\n",initError, TOFErrorBuffer); 
      timeOfFlightSensorAvailable = false;
    }
  }
*/

#ifdef ENABLE_OTA_AT_COMPILE_TIME
  if (enableOTAServer)
  {
    if (!setupOTAWebServerNotM5(ssid_1, password_1, label_1, timeout_1))
    {
      if (!setupOTAWebServerNotM5(ssid_2, password_2, label_2, timeout_2))
      {
        setupOTAWebServerNotM5(ssid_3, password_3, label_3, timeout_3);
      }
    }
  }
#endif

#ifdef ENABLE_ESPNOW_AT_COMPILE_TIME
  if (enableESPNow)
  {
    configAndStartUpESPNow();
  }
#endif

  while ( !amplifier.initI2S(I2S_AMP_BCLK_PIN, I2S_AMP_LRCLK_PIN, I2S_AMP_DIN_PIN) )
  {
   if (writeLogToSerial)
     USB_SERIAL.println("Init I2S Amplifier failed!");
    delay(3000);
  }

  if (writeLogToSerial)
    USB_SERIAL.println("Init I2S Amplifier succeeded");    
  
//  testFileIO();   // uncomment if issue with flash/SD

  while (!amplifier.initSDCard(SD_CS_PIN))
  {
    if (writeLogToSerial)
      USB_SERIAL.println("Initialize SD card for I2S Amplifier failed !");
    delay(3000);
  }

  if (writeLogToSerial)
     USB_SERIAL.println("Initialize SD Card by I2S Amplifier succeeded");
  
  amplifier.scanSDMusic(musicList);
  
  if (writeLogToSerial)
  {
    USB_SERIAL.println("Scan SD Card by I2S Amplifier for music list succeeded");
    printMusicList();
    USB_SERIAL.println("Print SD Card by I2S Amplifier music list contents succeeded");
  }
  
  amplifier.setVolume(defaultVolume);
  amplifier.closeFilter();

//  amplifier.playSDMusic("/_omen.wav"); // MBJ 13 Sep

//  amplifier.openFilter(bq_type_highpass, 500);
//  amplifier.SDPlayerControl(SD_AMPLIFIER_PLAY);

}

char mako_espnow_buffer[256];

void publishToMakoTestMessage(const char* testMessage)
{     
  if (isPairedWithMako && ESPNow_mako_peer.channel == ESPNOW_CHANNEL)
  {
    snprintf(mako_espnow_buffer,sizeof(mako_espnow_buffer),"S%s",testMessage);
    if (writeLogToSerial)
    {
      USB_SERIAL.println("Sending ESP S msg to Mako...");
      USB_SERIAL.println(mako_espnow_buffer);
    }
    
    ESPNowSendResult = esp_now_send(ESPNow_mako_peer.peer_addr, (uint8_t*)mako_espnow_buffer, strlen(mako_espnow_buffer)+1);
  }
}

char testMessageToMako[16]="";

void loop() 
{
  /* Light sensor and gesture sensor disabled
  if (lightSensorAvailable)
  {
    float lux=0.0;
    getLux(lux);
    if (writeLogToSerial)
     USB_SERIAL.printf("Lux Sensor: %f, ",lux);
  }

  if (timeOfFlightSensorAvailable)
  {
    // currently getting ...
    // 16:10:23.892 -> Error: Adafruit VL53L4CX Time Of Flight Sensor initialisation error: 2 Undefined enum
    // could be that I have a new firmware version of the sensor that has additional error codes
    // deal with it later.
    takeTimeOfFlightMeasurementAndWriteToSerial();
  }
*/
#if defined(ENABLE_PARSE_SERIAL)
  parseSerialCommand();
  delay(500);
#endif

  if (isPairedWithMako && millis() % 3000 == 0)
  {
    digitalWrite(beetleLed,HIGH);
    delay(100);
    digitalWrite(beetleLed,LOW);
    snprintf(testMessageToMako,sizeof(testMessageToMako),"%is uptime",millis() / 1000);
    publishToMakoTestMessage(testMessageToMako);
  }

  if (!isPairedWithMako && millis() % 10000 == 0)
  {
    digitalWrite(beetleLed,HIGH);
    delay(200);
    digitalWrite(beetleLed,LOW);
    delay(200);
    digitalWrite(beetleLed,HIGH);
    delay(200);
    digitalWrite(beetleLed,LOW);
    delay(200);
    digitalWrite(beetleLed,HIGH);
    delay(200);
    digitalWrite(beetleLed,LOW);
    // attempt to pair with mako every 10 seconds    
    isPairedWithMako = pairWithPeer(ESPNow_mako_peer,"Mako",1); // 1 connection attempts
    
    if (isPairedWithMako)
    {
      // send message to mako to say connection ok
      publishToMakoTestMessage("Conn Ok");
    }
  }
}

/*

// Light sensor and time of flight sensor disabled
void getLux(float &l)
{
  BH1750.start();         //starts a measurement
  l = BH1750.getLux();    //  waits until a conversion finished
}

void takeTimeOfFlightMeasurementAndWriteToSerial()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do 
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } 
  while (!NewDataReady);

  //Led on
  digitalWrite(beetleLed, HIGH);

  if ((!status) && (NewDataReady != 0)) 
  {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    
    if (writeLogToSerial)
    {
      snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
      USB_SERIAL.print(report);
      
      for (j = 0; j < no_of_object_found; j++) 
      {
        if (j != 0) 
        {
          USB_SERIAL.print("\r\n                               ");
        }
        
        USB_SERIAL.print("status=");
        USB_SERIAL.print(pMultiRangingData->RangeData[j].RangeStatus);
        USB_SERIAL.print(", D=");
        USB_SERIAL.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
        USB_SERIAL.print("mm");
        USB_SERIAL.print(", Signal=");
        USB_SERIAL.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
        USB_SERIAL.print(" Mcps, Ambient=");
        USB_SERIAL.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
        USB_SERIAL.print(" Mcps");
      }
      
      USB_SERIAL.println("");
    }
        
    if (status == 0) 
    {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }
  digitalWrite(beetleLed, LOW);
}

char* getTimeOfFlightSensorErrorString(VL53L4CX_Error e)
{
  switch (e)
  {
    case VL53L4CX_ERROR_NONE: return "VL53L4CX_ERROR_NONE";
    case VL53L4CX_ERROR_CALIBRATION_WARNING: return "VL53L4CX_ERROR_CALIBRATION_WARNING";
    case VL53L4CX_ERROR_MIN_CLIPPED: return "VL53L4CX_ERROR_MIN_CLIPPED";
    case VL53L4CX_ERROR_UNDEFINED: return "VL53L4CX_ERROR_UNDEFINED";
    case VL53L4CX_ERROR_INVALID_PARAMS: return "VL53L4CX_ERROR_INVALID_PARAMS";
    case VL53L4CX_ERROR_NOT_SUPPORTED: return "VL53L4CX_ERROR_NOT_SUPPORTED";
    case VL53L4CX_ERROR_RANGE_ERROR: return "VL53L4CX_ERROR_RANGE_ERROR";
    case VL53L4CX_ERROR_TIME_OUT: return "VL53L4CX_ERROR_TIME_OUT";
    case VL53L4CX_ERROR_MODE_NOT_SUPPORTED: return "VL53L4CX_ERROR_MODE_NOT_SUPPORTED";
    case VL53L4CX_ERROR_BUFFER_TOO_SMALL: return "VL53L4CX_ERROR_BUFFER_TOO_SMALL";
    case VL53L4CX_ERROR_COMMS_BUFFER_TOO_SMALL: return "VL53L4CX_ERROR_COMMS_BUFFER_TOO_SMALL";
    case VL53L4CX_ERROR_GPIO_NOT_EXISTING: return "VL53L4CX_ERROR_GPIO_NOT_EXIS";
    case VL53L4CX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED: return "VL53L4CX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED";
    case VL53L4CX_ERROR_CONTROL_INTERFACE: return "VL53L4CX_ERROR_CONTROL_INTERFACE";
    case VL53L4CX_ERROR_INVALID_COMMAND: return "VL53L4CX_ERROR_INVALID_COMMAND";
    case VL53L4CX_ERROR_DIVISION_BY_ZERO : return "VL53L4CX_ERROR_DIVISION_BY_ZERO";
    case VL53L4CX_ERROR_REF_SPAD_INIT: return "VL53L4CX_ERROR_REF_SPAD_INIT";
    case VL53L4CX_ERROR_GPH_SYNC_CHECK_FAIL: return "VL53L4CX_ERROR_GPH_SYNC_CHECK_FAIL";
    case VL53L4CX_ERROR_STREAM_COUNT_CHECK_FAIL: return "VL53L4CX_ERROR_STREAM_COUNT_CHECK_FAIL";
    case VL53L4CX_ERROR_GPH_ID_CHECK_FAIL: return "VL53L4CX_ERROR_GPH_ID_CHECK_FAIL";
    case VL53L4CX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL: return "VL53L4CX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL";
    case VL53L4CX_ERROR_ZONE_GPH_ID_CHECK_FAIL: return "VL53L4CX_ERROR_ZONE_GPH_ID_CHECK_FAIL";
    case VL53L4CX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_F";
    case VL53L4CX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL: return "VL53L4CX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_";
    case VL53L4CX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL";
    case VL53L4CX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL: return "VL53L4CX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL";
    case VL53L4CX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL: return "VL53L4CX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL";
    case VL53L4CX_ERROR_TUNING_PARM_KEY_MISMATCH: return "VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS: return "VL53L4CX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH: return "VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH";
    case VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW: return "VL53L4CX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW";
    case VL53L4CX_WARNING_OFFSET_CAL_MISSING_SAMPLES: return "VL53L4CX_WARNING_OFFSET_CAL_MISSING_SAMPLES";
    case VL53L4CX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH: return "VL53L4CX_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH";
    case VL53L4CX_WARNING_OFFSET_CAL_RATE_TOO_HIGH: return "VL53L4CX_WARNING_OFFSET_CAL_RATE_TOO_HI";
    case VL53L4CX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW: return "VL53L4CX_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW";
    case VL53L4CX_WARNING_ZONE_CAL_MISSING_SAMPLES: return "VL53L4CX_WARNING_ZONE_CAL_MISSING_SAMPLES";
    case VL53L4CX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH: return "VL53L4CX_WARNING_ZONE_CAL_SIGMA_TOO_HIGH";
    case VL53L4CX_WARNING_ZONE_CAL_RATE_TOO_HIGH: return "VL53L4CX_WARNING_ZONE_CAL_RATE_TOO_HIGH";
    case VL53L4CX_WARNING_XTALK_MISSING_SAMPLES: return "VL53L4CX_WARNING_XTALK_MISSING_SAMPLES";
    case VL53L4CX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT: return "VL53L4CX_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT";
    case VL53L4CX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT: return "VL53L4CX_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT";
    case VL53L4CX_ERROR_NOT_IMPLEMENTED: return "VL53L4CX_ERROR_NOT_IMPLEMENTED";
    case VL53L4CX_ERROR_PLATFORM_SPECIFIC_START: return "VL53L4CX_ERROR_PLATFORM_SPECIFIC_START";
    default: return "Undefined enum";
  }
}
*/

// ----- I WANT THIS CODE TO BE RUNNING FROM SD-card-API.c
// ----- but the Arduino IDE won't compile it... Help!
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    USB_SERIAL.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        USB_SERIAL.printf("Failed to open directory: %s\n",root);
        return;
    }
    if(!root.isDirectory()){
        USB_SERIAL.printf("Not a directory: %s\n",root);
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            USB_SERIAL.print("  DIR : ");
            USB_SERIAL.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            USB_SERIAL.print("  FILE: ");
            USB_SERIAL.print(file.name());
            USB_SERIAL.print("  SIZE: ");
            USB_SERIAL.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    USB_SERIAL.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        USB_SERIAL.printf("Dir created: %s\n",path);
    } else {
        USB_SERIAL.printf("mkdir failed: %s\n",path);
    }
}

void removeDir(fs::FS &fs, const char * path){
    USB_SERIAL.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        USB_SERIAL.printf("Dir removed: %s\n",path);
    } else {
        USB_SERIAL.printf("rmdir failed: %s\n",path);
    }
}

void readFile(fs::FS &fs, const char * path){
    USB_SERIAL.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        USB_SERIAL.printf("Failed to open file for reading: %s\n",path);
        return;
    }

    USB_SERIAL.printf("Read from file: %s\n",path);
    while(file.available()){
        USB_SERIAL.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    USB_SERIAL.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        USB_SERIAL.printf("Failed to open file for writing: %s\n",path);
        return;
    }
    if(file.print(message)){
        USB_SERIAL.printf("File written: %s\n",path);
    } else {
        USB_SERIAL.printf("Write failed: %s\n",path);
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    USB_SERIAL.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        USB_SERIAL.printf("Failed to open file for appending: %s\n",path);
        return;
    }
    if(file.print(message)){
        USB_SERIAL.printf("Message appended: %s\n",path);
    } else {
        USB_SERIAL.printf("Append failed: %s\n",path);
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    USB_SERIAL.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        USB_SERIAL.printf("File renamed: %s to %s\n",path1, path2);
    } else {
        USB_SERIAL.printf("Rename file failed: %s to %s\n",path1, path2);
    }
}

void deleteFile(fs::FS &fs, const char * path){
    USB_SERIAL.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        USB_SERIAL.printf("File deleted: %s\n",path);
    } else {
        USB_SERIAL.printf("Delete failed: %s\n",path);
    }
}

void testFileIO()
{
    USB_SERIAL.print("SD Card Type: ");
    
    if(cardType == CARD_MMC)
    {
        USB_SERIAL.println("MMC");
    } 
    else if(cardType == CARD_SD)
    {
        USB_SERIAL.println("SDSC");
    } 
    else if(cardType == CARD_SDHC)
    {
        USB_SERIAL.println("SDHC");
    } 
    else 
    {
        USB_SERIAL.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    USB_SERIAL.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFlashFileIO(SD, "/test.txt");
    USB_SERIAL.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    USB_SERIAL.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void testFlashFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        USB_SERIAL.printf("%u bytes read for %u ms file: %s\n",flen, end, path);
        file.close();
    } else {
        USB_SERIAL.printf("Failed to open file for reading: %s\n",path);
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        USB_SERIAL.printf("Failed to open file for writing: %s\n",path);
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    USB_SERIAL.printf("%u bytes written for %u ms file: %s\n", 2048 * 512, end, path);
    file.close(); 
}

void toggleOTAActive()
{
  #ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME

   if (otaActiveListening)
   {
     asyncWebServer.end();
     USB_SERIAL.println("OTA Disabled");
     otaActiveListening=false;
   }
   else
   {
     if (WiFi.status() == WL_CONNECTED)
     {
       asyncWebServer.begin();
       USB_SERIAL.printf("OTA Enabled");
       otaActiveListening=true;
     }
     else
     {
       USB_SERIAL.println("Error: Enable Wifi First");
     }
   }  

   #endif
}

void toggleWiFiActive()
{
  #ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
   if (WiFi.status() == WL_CONNECTED)
   {
      if (otaActiveListening)
      {
         asyncWebServer.end();
         USB_SERIAL.println("OTA Disabled");
         otaActiveListening=false;
      }

       WiFi.disconnect();
       USB_SERIAL.printf("Wifi Disabled");
   }
   else
   {
     USB_SERIAL.printf("Wifi Connecting");

    // startup Wifi only
    if (!connectWiFiNoOTANotM5(ssid_1, password_1, label_1, timeout_1))
    {
      if (!connectWiFiNoOTANotM5(ssid_2, password_2, label_2, timeout_2))
      {
        connectWiFiNoOTANotM5(ssid_3, password_3, label_3, timeout_3);
      }
    }
   }
  #endif
}


bool connectWiFiNoOTANotM5(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
#ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME

  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  USB_SERIAL.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    USB_SERIAL.print(".");
    delay(500);
  }
  USB_SERIAL.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    USB_SERIAL.printf("%s\n\n",WiFi.localIP().toString());
    USB_SERIAL.println(WiFi.macAddress());
    connected = true;
  }

  delay(1000);
  return connected;
#else
  return false;
#endif
}


bool setupOTAWebServerNotM5(const char* _ssid, const char* _password, const char* label, uint32_t timeout)
{
#ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME

  bool connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  // Wait for connection for max of timeout/1000 seconds
  USB_SERIAL.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    USB_SERIAL.print(".");
    delay(500);
  }
  USB_SERIAL.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "To upload firmware use /update");
    });

    AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
    asyncWebServer.begin();

    USB_SERIAL.printf("%s\n\n",WiFi.localIP().toString());
    USB_SERIAL.println(WiFi.macAddress());
    connected = true;

  }
  delay(1000);
  return connected;
#else
  return false;
#endif
}

// ----- I WANT THIS CODE TO BE RUNNING FROM MAX-AMP-API.c

/**************************************************************
              Print the list of the scanned music files that can be played                  
**************************************************************/

void printMusicList(void)
{
  uint8_t i = 0;
  if(musicList[i].length()){
    USB_SERIAL.println("\nMusic List: ");
  }else{
    USB_SERIAL.println("The SD card audio file scan is empty, please check whether there are audio files in the SD card that meet the format!");
  }

  while(musicList[i].length()){
    USB_SERIAL.print("\t");
    USB_SERIAL.print(i);
    USB_SERIAL.print("  -  ");
    USB_SERIAL.println(musicList[i]);
    i++;
  }
}

/**************************************************************
                    The parsing and implementation of the serial command                        
**************************************************************/

void parseSerialCommand(void)
{
  String cmd;   // Save the command type read in the serial port
  float value;   // Save the command value read in the serial port

  /**
   * Command format: cmd-value
   * cmd : indicate the command type
   * value : indicate the set value corresponding to the command type, some commands can be empty
   * For example: (1) set high-pass filter, filter the audio data below 500: hp-500
   *      (2) close filter: closeFilter-
   */
  if(USB_SERIAL.available()){   // Detect whether there is an available serial command
    cmd = USB_SERIAL.readStringUntil('-');   // Read the specified terminator character string, used to cut and identify the serial command.  The same comment won't repeat later.

    if(cmd.equals("hp")){   // Determine if it’s the command type for setting high-pass filter
      USB_SERIAL.println("Setting a High-Pass filter...\n");
      value =USB_SERIAL.parseFloat();   // Parse character string and return floating point number

      /**
       * @brief Open audio filter
       * @param type - bq_type_highpass: open high-pass filtering; bq_type_lowpass: open low-pass filtering
       * @param fc - Threshold of filtering, range: 2-20000
       * @note For example, setting high-pass filter mode and the threshold of 500 indicates to filter out the audio signal below 500; high-pass filter and low-pass filter can work simultaneously.
       */
      amplifier.openFilter(bq_type_highpass, value);


    }else if(cmd.equals("lp")){   // Determine if it's the command type for setting low-pass filter
      USB_SERIAL.println("Setting a Low-Pass filter...\n");
      value =USB_SERIAL.parseFloat();

      amplifier.openFilter(bq_type_lowpass, value);

    }else if(cmd.equals("closeFilter")){   // Determine if it's the command type for closing filter
      USB_SERIAL.println("Closing filter...\n");

      /**
       * @brief Close the audio filter
       */
      amplifier.closeFilter();

    }else if(cmd.equals("vol")){   // Determine if it's the command type for setting volume
      USB_SERIAL.println("Setting volume...\n");
      value =USB_SERIAL.parseFloat();

      /**
       * @brief Set volume
       * @param vol - Set volume, the range can be set to 0-9
       * @note 5 for the original volume of audio data, no increase or decrease
       */
      amplifier.setVolume(value);

    }else if(cmd.equals("start")){   // Determine if it's the command type for starting playback
      USB_SERIAL.println("starting amplifier...\n");

      /**
       * @brief SD card music playback control interface
       * @param CMD - Playback control command: 
       * @n SD_AMPLIFIER_PLAY: Start to play music, which can be played from the position where you paused before
       * @n   If no music file is selected through playSDMusic(), the first one in the list will be played by default.
       * @n   Playback error may occur if music files are not scanned from SD card in the correct format (only support English for path name of music files and WAV for their format currently)
       * @n SD_AMPLIFIER_PAUSE: pause playback, keep the playback position of the current music file
       * @n SD_AMPLIFIER_STOP: stop playback, end the current music playback
       * @return None
       */
      amplifier.SDPlayerControl(SD_AMPLIFIER_PLAY);

    }else if(cmd.equals("pause")){   // Determine if it's the command type for pausing playback
      USB_SERIAL.println("Pause amplifier...\n");

      // The same as above
      amplifier.SDPlayerControl(SD_AMPLIFIER_PAUSE);

    }else if(cmd.equals("stop")){   // Determine if it's the command type for stopping playback
      USB_SERIAL.println("Stopping amplifier...\n");

      // The same as above
      amplifier.SDPlayerControl(SD_AMPLIFIER_STOP);

    }else if(cmd.equals("musicList")){   // Determine if it's the command type for printing the list of the music files that can be played currently
      USB_SERIAL.println("Scanning music list...\n");

      /**
       * @brief Scan the music files in WAV format in the SD card
       * @param musicList - The music files in .wav format scanned from the SD card. Type is character string array
       * @return None
       * @note Only support English for music file and path name currently and try to avoid spaces, only support .wav for the audio format currently
       */
      amplifier.scanSDMusic(musicList);
      /**
       * Print the list of the scanned music files that can be played
       */
      printMusicList();

    }else if(cmd.equals("changeMusic")){   // Determine if it's the command type for changing songs according to the music list
      cmd = musicList[USB_SERIAL.parseInt()];

      /**
       * @brief Play music files in the SD card
       * @param Filename - music file name, only support the music files in .wav format currently
       * @note Music file name must be an absolute path like /musicDir/music.wav
       * @return None
       */
      if(cmd.length()){
        USB_SERIAL.println("Changing Music...\n");
        amplifier.playSDMusic(cmd.c_str());
      }else{
        USB_SERIAL.println("The currently selected music file is incorrect!\n");
      }

    }else{   // Unknown command type
      USB_SERIAL.println("Help : \n \
      Currently available commands (format: cmd-value):\n \
        Start playback: e.g. start-\n \
        Pause playback: e.g. pause-\n \
        Stop playback: e.g. stop-\n \
        Print music list: e.g. musicList-\n \
        Change songs according to the music list: e.g. changeMusic-1\n \
        Set and open high-pass filter: e.g. hp-500\n \
        Set and open low-pass filter: e.g. lp-15000\n \
        Close filter: e.g. closeFilter-\n \
        Set volume: e.g. vol-5.0\n \
      For the detailed meaning, please refer to the code comments of this demo.\n");   //
    }
    while(USB_SERIAL.read() >= 0);   // Clear the remaining data in the serial port
  }
}

#ifdef ENABLE_ESPNOW_AT_COMPILE_TIME

void OnESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowMessagesDelivered++;
  }
  else
  {
    ESPNowMessagesFailedToDeliver++;
  }
}

char ESPNowbuffer[100];

// callback when data is recv from Master
void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) 
{
  if (writeLogToSerial)
  {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    USB_SERIAL.printf("Last Packet Recv from: %s\n",macStr);
    USB_SERIAL.printf("Last Packet Recv 1st Byte: '%c'\n",*data);
    USB_SERIAL.printf("Last Packet Recv Length: %d\n",data_len);
  }
  
  const char* curTrackName = amplifier.getTrackFilename(currentTrack);

  switch (*data)
  {
    case '0':
    case '1':   
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case ':':
    case ';':
    case '<':
    case '=':
    case '>':
    case '?':
    case '@':
    {           // allow up to 17 randomly chosen sounds
      if (millis() > lastAudioGuidancePlayedAt + audioGuidanceMinimumGap)
      {
        if (writeLogToSerial)
           USB_SERIAL.printf("1.Command %c... Play track\n",*data);

        lastAudioGuidancePlayedAt = millis();

        currentTrack=(*data) - '0';
        currentTrackFilename = amplifier.getTrackFilename(currentTrack);

        if (writeLogToSerial)
          USB_SERIAL.printf("Command %c... Play track %d %s\n",*data,currentTrack,currentTrackFilename);

        amplifier.playSDMusic(currentTrackFilename);
      }
      else
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Command %c... wait for minimum gap\n",*data);
      }
      break;
    }
    case 'A': // toggle playback
    {
      if (amplifier.getAmplifierState() == SD_AMPLIFIER_PLAY)
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Command A... Pause Track %d %s\n",currentTrack,curTrackName);
        amplifier.SDPlayerControl(SD_AMPLIFIER_PAUSE);
      }
      else if (amplifier.getAmplifierState() == SD_AMPLIFIER_STOP)
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Command A... Unstop/Play Track %d %s\n",currentTrack,curTrackName);
        amplifier.SDPlayerControl(SD_AMPLIFIER_PLAY);
      }
      else if (amplifier.getAmplifierState() == SD_AMPLIFIER_PAUSE)
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Command A... Unpause Track %d %s\n",currentTrack,curTrackName);
        amplifier.SDPlayerControl(SD_AMPLIFIER_PLAY);
      }
      else
      {
        if (writeLogToSerial)
          USB_SERIAL.printf("Command A... Pause/? Track %d %s\n",currentTrack,curTrackName);
        amplifier.SDPlayerControl(SD_AMPLIFIER_PAUSE);
      }
      
      break;
    }
          
    case 'B': // cycle volume up
    {
      volume = (volume >= maxVolume ? minVolume : volume+1);
      amplifier.setVolume(volume);
      if (writeLogToSerial)
        USB_SERIAL.printf("Command B... Cycle Volume Up: %f\n",volume);
      break;
    }
    
    case 'C': // skip to next track
    {
      amplifier.playSDMusic(musicList[getNextTrack()].c_str());
      if (writeLogToSerial)
        USB_SERIAL.printf("Command C... Skip to next track %d %s\n",currentTrack,amplifier.getTrackFilename(currentTrack));
      break;
    }
    
    case 'D': // stop playback
    {
      amplifier.SDPlayerControl(SD_AMPLIFIER_STOP);
      if (writeLogToSerial)
        USB_SERIAL.printf("Command D... Stop playback\n");
      break;
    }
    
    case 'E': // set volume
    {
      if (data_len == 2)
      {
        // volume is in the second byte
        volume = *(data+1);
        amplifier.setVolume(volume);
        if (writeLogToSerial)
          USB_SERIAL.printf("Command E... Set Volume: %f\n",volume);
      }
      else
      {
        // do nothing
      }
    }
    default:
      break;
  }
}

uint8_t getNextTrack()
{
  if (++currentTrack >= amplifier.getTrackCount())
    currentTrack = 0;

  return currentTrack;
}

void InitESPNow() 
{
  WiFi.disconnect();
  
  if (esp_now_init() == ESP_OK) 
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Success");
    ESPNowActive = true;
  }
  else 
  {
    if (writeLogToSerial)
      USB_SERIAL.println("ESPNow Init Failed");
    ESPNowActive = false;
  }
}

void configESPNowDeviceAP() 
{
  String Prefix = "AudioPod:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), ESPNOW_CHANNEL, 0);

  if (writeLogToSerial)
  {
    if (!result) 
    {
      USB_SERIAL.println("AP Config failed.");
    } 
    else 
    {
      USB_SERIAL.printf("AP Config Success. Broadcasting with AP: %s\n",String(SSID).c_str());
      USB_SERIAL.printf("WiFi Channel: %d\n",WiFi.channel());
    }
  }  

  // Init ESPNow with a fallback logic
  InitESPNow();
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnESPNowDataRecv);
  esp_now_register_send_cb(OnESPNowDataSent);
}

void configAndStartUpESPNow()
{
  if (writeLogToSerial)
    USB_SERIAL.println("ESPNow/Basic Example");
  
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  
  // configure device AP mode
  configESPNowDeviceAP();
  
  // This is the mac address of this peer in AP Mode
  if (writeLogToSerial)
    USB_SERIAL.print("AP MAC: "); USB_SERIAL.println(WiFi.softAPmacAddress()); 
}

bool TeardownESPNow()
{
  bool result = false;

  if (enableESPNow && ESPNowActive)
  {
    WiFi.disconnect();
    ESPNowActive = false;
    result = true;
  }
  
  return result;
}

// Scan for peers in AP mode
bool ESPNowScanForPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix)
{
  bool peerFound = false;

  if (writeLogToSerial)
    USB_SERIAL.println("Scanning Networks...");

  int8_t scanResults = WiFi.scanNetworks();
  if (writeLogToSerial)
    USB_SERIAL.println("Complete");
  
  // reset on each scan 
  memset(&peer, 0, sizeof(peer));

  if (writeLogToSerial)
    USB_SERIAL.println("");

  if (scanResults == 0) 
  {   
    if (writeLogToSerial)
      USB_SERIAL.println("No WiFi devices in AP Mode found");

    peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
  } 
  else 
  {
    if (writeLogToSerial)
    {
      USB_SERIAL.print("Found "); USB_SERIAL.print(scanResults); USB_SERIAL.println(" devices ");
    }
    
    for (int i = 0; i < scanResults; ++i) 
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (writeLogToSerial && ESPNOW_PRINTSCANRESULTS) 
      {
        USB_SERIAL.print(i + 1);
        USB_SERIAL.print(": ");
        USB_SERIAL.print(SSID);
        USB_SERIAL.print(" (");
        USB_SERIAL.print(RSSI);
        USB_SERIAL.print(")");
        USB_SERIAL.println("");
      }
      
      delay(10);
      
      // Check if the current device starts with the peerSSIDPrefix
      if (SSID.indexOf(peerSSIDPrefix) == 0) 
      {
        if (writeLogToSerial)
        {
          // SSID of interest
          USB_SERIAL.println("Found a peer.");
          USB_SERIAL.print(i + 1); USB_SERIAL.print(": "); USB_SERIAL.print(SSID); USB_SERIAL.print(" ["); USB_SERIAL.print(BSSIDstr); USB_SERIAL.print("]"); USB_SERIAL.print(" ("); USB_SERIAL.print(RSSI); USB_SERIAL.print(")"); USB_SERIAL.println("");
        }
                
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) 
        {
          for (int ii = 0; ii < 6; ++ii ) 
          {
            peer.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        peer.channel = ESPNOW_CHANNEL; // pick a channel
        peer.encrypt = 0; // no encryption

        peer.priv = (void*)peerSSIDPrefix;   // distinguish between different peers

        peerFound = true;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (peerFound) 
  {
    if (writeLogToSerial)
      USB_SERIAL.println("Peer Found");
  } 
  else 
  {
    if (writeLogToSerial)
      USB_SERIAL.println("Peer Not Found");
  }
  
  // clean up ram
  WiFi.scanDelete();

  return peerFound;
}

bool pairWithPeer(esp_now_peer_info_t& peer, const char* peerSSIDPrefix, int maxAttempts)
{
  bool isPaired = false;
  while(maxAttempts-- && !isPaired)
  {
    bool result = ESPNowScanForPeer(peer,peerSSIDPrefix);

    // check if peer channel is defined
    if (result && peer.channel == ESPNOW_CHANNEL)
    { 
      isPaired = ESPNowManagePeer(peer);
      if (writeLogToSerial)
        USB_SERIAL.printf("%s Pair ok\n",peerSSIDPrefix);
    }
    else
    {
      peer.channel = ESPNOW_NO_PEER_CHANNEL_FLAG;
      if (writeLogToSerial)
        USB_SERIAL.printf("%s Pair fail\n",peerSSIDPrefix);
    }
  }

  delay(1000);
    
  return isPaired;
}

// Check if the peer is already paired with the master.
// If not, pair the peer with master
bool ESPNowManagePeer(esp_now_peer_info_t& peer)
{
  bool result = false;
  
  if (peer.channel == ESPNOW_CHANNEL) 
  {
    if (ESPNOW_DELETEBEFOREPAIR) 
    {
      ESPNowDeletePeer(peer);
    }

    if (writeLogToSerial)
      USB_SERIAL.print("Peer Status: ");
      
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer.peer_addr);
    
    if (exists) 
    {
      // Peer already paired.
      if (writeLogToSerial)
        USB_SERIAL.println("Already Paired");

      result = true;
    } 
    else 
    {
      // Peer not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&peer);
      
      if (addStatus == ESP_OK) 
      {
        // Pair success
        if (writeLogToSerial)
          USB_SERIAL.println("Pair success");
        result = true;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
        if (writeLogToSerial)
          USB_SERIAL.println("ESPNOW Not Init");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_ARG) 
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Invalid Argument");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_FULL) 
      {
        if (writeLogToSerial)
            USB_SERIAL.println("Peer list full");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Out of memory");
        result = false;
      } 
      else if (addStatus == ESP_ERR_ESPNOW_EXIST) 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Peer Exists");
        result = true;
      } 
      else 
      {
        if (writeLogToSerial)
          USB_SERIAL.println("Not sure what happened");
        result = false;
      }
    }
  }
  else 
  {
    // No peer found to process
    if (writeLogToSerial)
      USB_SERIAL.println("No Peer found to process");
    
    result = false;
  }

  return result;
}

void ESPNowDeletePeer(esp_now_peer_info_t& peer) 
{
  if (peer.channel != ESPNOW_NO_PEER_CHANNEL_FLAG)
  {
    esp_err_t delStatus = esp_now_del_peer(peer.peer_addr);
    
    if (writeLogToSerial)
    {
      USB_SERIAL.print("Peer Delete Status: ");
      if (delStatus == ESP_OK) 
      {
        // Delete success
        USB_SERIAL.println("ESPNowDeletePeer::Success");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) 
      {
        // How did we get so far!!
        USB_SERIAL.println("ESPNowDeletePeer::ESPNOW Not Init");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_ARG) 
      {
        USB_SERIAL.println("ESPNowDeletePeer::Invalid Argument");
      } 
      else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) 
      {
        USB_SERIAL.println("ESPNowDeletePeer::Peer not found.");
      } 
      else 
      {
        USB_SERIAL.println("Not sure what happened");
      }
    }
  }
}
 
#endif
