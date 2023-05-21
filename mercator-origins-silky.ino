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

#include <DFRobot_MAX98357A.h>

#include "mercator_secrets.c"

// I2C Light Sensor
// see https://github.com/Starmbi/hp_BH1750
#include <hp_BH1750.h>
hp_BH1750 BH1750;

// I2C Time of Flight sensor. Adafruit VL53L4CX
// see https://learn.adafruit.com/adafruit-vl53l4cx-time-of-flight-distance-sensor
#include <vl53l4cx_class.h>
#define DEV_I2C Wire
#define XSHUT_PIN A1
#define VL53L4CX_DEFAULT_DEVICE_ADDRESS 0x12

VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, XSHUT_PIN);

//#define ENABLE_ELEGANT_OTA_AT_COMPILE_TIME
//#define ENABLE_ESPNOW_AT_COMPILE_TIME

const bool enableOTAServer = false;          // over the air updates

const bool enableESPNow = false;//!enableOTAServer; // cannot have OTA server on regular wifi and espnow concurrently running
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
  const uint8_t ESPNOW_CHANNEL=1;
#endif

const int beetleLed = 10;
const uint8_t I2S_AMP_BCLK_PIN = GPIO_NUM_0;
const uint8_t I2S_AMP_LRCLK_PIN = GPIO_NUM_1;
const uint8_t I2S_AMP_DIN_PIN = GPIO_NUM_7;

const uint8_t SD_CHIP_SELECT_PIN = GPIO_NUM_2;

uint8_t cardType = 0;

String musicList[100];   // SD card music playlist 

DFRobot_MAX98357A amplifier;   

void setup()
{
  pinMode(beetleLed,OUTPUT);

  // LED flash - we're alive!
  Serial.begin(115200);
  int warmUp=20;
  
  while (warmUp--)
  {
    digitalWrite(beetleLed,HIGH);
    delay(250);
    digitalWrite(beetleLed,LOW);
    delay(250);
   
    Serial.println("Warming up...");
  }
  Serial.println("\nHere we go...");

  while(true)
  {
    if(!SD.begin(SD_CHIP_SELECT_PIN))
    {
        Serial.println("Card Mount Failed");
        delay(2000);
    }
    else
    {
      Serial.println("Card Mount Succeeded");
      break;
    }
  }
  
  cardType = SD.cardType();

  if(cardType == CARD_NONE)
  {
      Serial.println("No SD card attached");
      return;
  }

  DEV_I2C.begin();

  if (enableLightSensor)
  {
    lightSensorAvailable = BH1750.begin(BH1750_TO_GROUND);// init the sensor with address pin connetcted to ground
                                              // result (bool) wil be be "false" if no sensor found
    if (!lightSensorAvailable) 
    {
      Serial.println("No BH1750 sensor found!");
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
      Serial.println("Adafruit VL53L4CX Time Of Flight Sensor Initialised ok");

      VL53L4CX_Error measureError = sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();  

      if (measureError == VL53L4CX_ERROR_NONE)
      {
        Serial.println("Adafruit VL53L4CX Time Of Flight Sensor Started Measurement ok");
        timeOfFlightSensorAvailable = true;
      }
      else
      {
        const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(measureError);
        Serial.printf("\nError: Adafruit VL53L4CX Time Of Flight Sensor startup measurement error: %i %s\n",measureError, TOFErrorBuffer);
        timeOfFlightSensorAvailable = false;
      }
    }
    else
    {
      const char* TOFErrorBuffer = getTimeOfFlightSensorErrorString(initError);
      Serial.printf("Error: Adafruit VL53L4CX Time Of Flight Sensor initialisation error: %i %s\n",initError, TOFErrorBuffer); 
      timeOfFlightSensorAvailable = false;
    }
  }

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
    Serial.println("Init I2S Amplifier failed!");
    delay(3000);
  }

  Serial.println("Init I2S Amplifier succeeded");    
  
  testFileIO();

  while (!amplifier.initSDCard(SD_CHIP_SELECT_PIN))
  {
    Serial.println("Initialize SD card for I2S Amplifier failed !");
    delay(3000);
  }

  Serial.println("Initialize SD Card by I2S Amplifier succeeded");
  
  amplifier.scanSDMusic(musicList);
  
  Serial.println("Scan SD Card by I2S Amplifier for music list succeeded");

  printMusicList();
  
  Serial.println("Print SD Card by I2S Amplifier music list contents succeeded");

  amplifier.setVolume(5);
  amplifier.closeFilter();
  amplifier.openFilter(bq_type_highpass, 500);
  amplifier.SDPlayerControl(SD_AMPLIFIER_PLAY);

  Serial.println("I2S Amplifier play music file from SD Card function completed");

  if(musicList[1].length())
  {
    Serial.println("Changing Music...\n");
    amplifier.playSDMusic(musicList[1].c_str());
  }
  else
  {
    Serial.println("The currently selected music file is incorrect!\n");
  }
}


void loop() 
{
/*
  if (lightSensorAvailable)
  {
    float lux=0.0;
    getLux(lux);
    Serial.printf("Lux Sensor: %f, ",lux);
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

  parseSerialCommand();
  delay(500);
}




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
    
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    Serial.print(report);
    
    for (j = 0; j < no_of_object_found; j++) 
    {
      if (j != 0) 
      {
        Serial.print("\r\n                               ");
      }
      
      Serial.print("status=");
      Serial.print(pMultiRangingData->RangeData[j].RangeStatus);
      Serial.print(", D=");
      Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      Serial.print("mm");
      Serial.print(", Signal=");
      Serial.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      Serial.print(" Mcps, Ambient=");
      Serial.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      Serial.print(" Mcps");
    }
    
    Serial.println("");
    
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

// ----- I WANT THIS CODE TO BE RUNNING FROM SD-card-API.c
// ----- but the Arduino IDE won't compile it... Help!
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.printf("Failed to open directory: %s\n",root);
        return;
    }
    if(!root.isDirectory()){
        Serial.printf("Not a directory: %s\n",root);
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.printf("Dir created: %s\n",path);
    } else {
        Serial.printf("mkdir failed: %s\n",path);
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.printf("Dir removed: %s\n",path);
    } else {
        Serial.printf("rmdir failed: %s\n",path);
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.printf("Failed to open file for reading: %s\n",path);
        return;
    }

    Serial.printf("Read from file: %s\n",path);
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.printf("Failed to open file for writing: %s\n",path);
        return;
    }
    if(file.print(message)){
        Serial.printf("File written: %s\n",path);
    } else {
        Serial.printf("Write failed: %s\n",path);
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.printf("Failed to open file for appending: %s\n",path);
        return;
    }
    if(file.print(message)){
        Serial.printf("Message appended: %s\n",path);
    } else {
        Serial.printf("Append failed: %s\n",path);
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.printf("File renamed: %s to %s\n",path1, path2);
    } else {
        Serial.printf("Rename file failed: %s to %s\n",path1, path2);
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.printf("File deleted: %s\n",path);
    } else {
        Serial.printf("Delete failed: %s\n",path);
    }
}

void testFileIO()
{
    Serial.print("SD Card Type: ");
    
    if(cardType == CARD_MMC)
    {
        Serial.println("MMC");
    } 
    else if(cardType == CARD_SD)
    {
        Serial.println("SDSC");
    } 
    else if(cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    } 
    else 
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

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
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
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
        Serial.printf("%u bytes read for %u ms file: %s\n",flen, end, path);
        file.close();
    } else {
        Serial.printf("Failed to open file for reading: %s\n",path);
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.printf("Failed to open file for writing: %s\n",path);
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms file: %s\n", 2048 * 512, end, path);
    file.close(); 
}

void toggleOTAActive()
{
  #ifdef ENABLE_ELEGANT_OTA_AT_COMPILE_TIME

   if (otaActiveListening)
   {
     asyncWebServer.end();
     Serial.println("OTA Disabled");
     otaActiveListening=false;
   }
   else
   {
     if (WiFi.status() == WL_CONNECTED)
     {
       asyncWebServer.begin();
       Serial.printf("OTA Enabled");
       otaActiveListening=true;
     }
     else
     {
       Serial.println("Error: Enable Wifi First");
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
         Serial.println("OTA Disabled");
         otaActiveListening=false;
      }

       WiFi.disconnect();
       Serial.printf("Wifi Disabled");
   }
   else
   {
     Serial.printf("Wifi Connecting");

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
  Serial.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    // check for cancellation button - top button.
    Serial.print(".");
    delay(500);
  }
  Serial.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("%s\n\n",WiFi.localIP().toString());
    Serial.println(WiFi.macAddress());
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
  Serial.printf("%s Wifi", label);
  int count = timeout / 500;
  while (WiFi.status() != WL_CONNECTED && --count > 0)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\n\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "To upload firmware use /update");
    });

    AsyncElegantOTA.begin(&asyncWebServer);    // Start AsyncElegantOTA
    asyncWebServer.begin();

    Serial.printf("%s\n\n",WiFi.localIP().toString());
    Serial.println(WiFi.macAddress());
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
    Serial.println("\nMusic List: ");
  }else{
    Serial.println("The SD card audio file scan is empty, please check whether there are audio files in the SD card that meet the format!");
  }

  while(musicList[i].length()){
    Serial.print("\t");
    Serial.print(i);
    Serial.print("  -  ");
    Serial.println(musicList[i]);
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
  if(Serial.available()){   // Detect whether there is an available serial command
    cmd = Serial.readStringUntil('-');   // Read the specified terminator character string, used to cut and identify the serial command.  The same comment won't repeat later.

    if(cmd.equals("hp")){   // Determine if it’s the command type for setting high-pass filter
      Serial.println("Setting a High-Pass filter...\n");
      value =Serial.parseFloat();   // Parse character string and return floating point number

      /**
       * @brief Open audio filter
       * @param type - bq_type_highpass: open high-pass filtering; bq_type_lowpass: open low-pass filtering
       * @param fc - Threshold of filtering, range: 2-20000
       * @note For example, setting high-pass filter mode and the threshold of 500 indicates to filter out the audio signal below 500; high-pass filter and low-pass filter can work simultaneously.
       */
      amplifier.openFilter(bq_type_highpass, value);


    }else if(cmd.equals("lp")){   // Determine if it's the command type for setting low-pass filter
      Serial.println("Setting a Low-Pass filter...\n");
      value =Serial.parseFloat();

      amplifier.openFilter(bq_type_lowpass, value);

    }else if(cmd.equals("closeFilter")){   // Determine if it's the command type for closing filter
      Serial.println("Closing filter...\n");

      /**
       * @brief Close the audio filter
       */
      amplifier.closeFilter();

    }else if(cmd.equals("vol")){   // Determine if it's the command type for setting volume
      Serial.println("Setting volume...\n");
      value =Serial.parseFloat();

      /**
       * @brief Set volume
       * @param vol - Set volume, the range can be set to 0-9
       * @note 5 for the original volume of audio data, no increase or decrease
       */
      amplifier.setVolume(value);

    }else if(cmd.equals("start")){   // Determine if it's the command type for starting playback
      Serial.println("starting amplifier...\n");

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
      Serial.println("Pause amplifier...\n");

      // The same as above
      amplifier.SDPlayerControl(SD_AMPLIFIER_PAUSE);

    }else if(cmd.equals("stop")){   // Determine if it's the command type for stopping playback
      Serial.println("Stopping amplifier...\n");

      // The same as above
      amplifier.SDPlayerControl(SD_AMPLIFIER_STOP);

    }else if(cmd.equals("musicList")){   // Determine if it's the command type for printing the list of the music files that can be played currently
      Serial.println("Scanning music list...\n");

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
      cmd = musicList[Serial.parseInt()];

      /**
       * @brief Play music files in the SD card
       * @param Filename - music file name, only support the music files in .wav format currently
       * @note Music file name must be an absolute path like /musicDir/music.wav
       * @return None
       */
      if(cmd.length()){
        Serial.println("Changing Music...\n");
        amplifier.playSDMusic(cmd.c_str());
      }else{
        Serial.println("The currently selected music file is incorrect!\n");
      }

    }else{   // Unknown command type
      Serial.println("Help : \n \
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
    while(Serial.read() >= 0);   // Clear the remaining data in the serial port
  }
}

// Integrate ESPNow slave code

#ifdef ENABLE_ESPNOW_AT_COMPILE_TIME

char ESPNowbuffer[100];

// callback when data is recv from Master
void OnESPNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: \n%s",macStr);
  Serial.printf("Last Packet Recv Data: '%c'\n",*data);
}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void configESPNowDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), ESPNOW_CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void configAndStartUpESPNow()
{
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configESPNowDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnESPNowDataRecv);
}

#endif
