/**
/*
 ****************************************************************
 * @file  : DownloadFile_hexparsing.ino
 * @author  : Mostafa Ahmed
 * @brief : Telematics code
 ****************************************************************
 */

/* Firebase */
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif __has_include(<WiFiNINA.h>)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>)
#include <WiFiS3.h>
#endif

#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the SD card interfaces setting and mounting
#include <addons/SDHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "FOTA"
#define WIFI_PASSWORD "PASSWORD"

/* 2. Define the API Key */
#define API_KEY "AIzaSyDCTg0LFB22lCWNI4QAv2D2jJPdzF1wl2s"

/* 3. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "mostafa.mostafa@ejust.edu.eg"
#define USER_PASSWORD "123456789"

/* 4. Define the Firebase storage bucket ID e.g bucket-name.appspot.com */
#define STORAGE_BUCKET_ID "fota-and-remote-diagnostics.appspot.com"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

bool taskCompleted = false;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif

/* SPI */
#include <SPI.h>

#define SS_PIN 15  // Slave Select (SS) pin

// Define your SPI settings
SPISettings mySettings(20000000, MSBFIRST, SPI_MODE0);  // Adjust the settings based on your requirements
uint8_t acknowledgment = 0;
int x = 0;

int repeat = 0; // 0 Repeat  --  1 Done SPI send -- 2 Done erase sector
/* file system*/ 
char buf[2] = {0};
char packet[6478]; // 1 length + 1 command + 4 address + 1 payload length + 6468 payload + 4 CRC 

void setup()
{
/* SPI */
    SPI.begin();
/* File system*/
    LittleFS.begin();
  
/* Firebase */
    Serial.begin(115200);

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    multi.addAP(WIFI_SSID, WIFI_PASSWORD);
    multi.run();
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

    Serial.print("Connecting to Wi-Fi");
    unsigned long ms = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
        if (millis() - ms > 10000)
            break;
#endif
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // The WiFi credentials are required for Pico W
    // due to it does not have reconnect feature.
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    config.wifi.clearAP();
    config.wifi.addAP(WIFI_SSID, WIFI_PASSWORD);
#endif

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectNetwork(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    /* Assign download buffer size in byte */
    // Data to be downloaded will read as multiple chunks with this size, to compromise between speed and memory used for buffering.
    // The memory from external SRAM/PSRAM will not use in the TCP client internal rx buffer.
    config.fcs.download_buffer_size = 2048;

    Firebase.begin(&config, &auth);

    // if use SD card, mount it.
    SD_Card_Mounting(); // See src/addons/SDHelper.h
}

// The Firebase Storage download callback function
void fcsDownloadCallback(FCS_DownloadStatusInfo info)
{
    if (info.status == firebase_fcs_download_status_init)
    {
        Serial.printf("Downloading file %s (%d) to %s\n", info.remoteFileName.c_str(), info.fileSize, info.localFileName.c_str());
    }
    else if (info.status == firebase_fcs_download_status_download)
    {
        Serial.printf("Downloaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    }
    else if (info.status == firebase_fcs_download_status_complete)
    {
        Serial.println("Download completed\n");
    }
    else if (info.status == firebase_fcs_download_status_error)
    {
        Serial.printf("Download failed, %s\n", info.errorMsg.c_str());
    }
}

void loop()
{

    // Firebase.ready() should be called repeatedly to handle authentication tasks.

    if (Firebase.ready() && !taskCompleted)
    {
        taskCompleted = true;

        Serial.println("\nDownload file...\n");

        // The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
        if (!Firebase.Storage.download(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, "app_nucleo_output.txt" /* path of remote file stored in the bucket */, "/testBinaryOutput.txt" /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, fcsDownloadCallback /* callback function */))
            Serial.println(fbdo.errorReason());
            }
    Hex_Parsing();
    erase_sector();
    SPIsend();
    Serial.println (repeat);
    if ( 4 == repeat ) { 
      SPI.beginTransaction(mySettings);
      digitalWrite(SS_PIN, LOW);
      acknowledgment = SPI.transfer(0x0); // erase 
      Serial.println("Sent: 0x0 ");
      Serial.println(acknowledgment);
      digitalWrite(SS_PIN, HIGH);
      SPI.endTransaction();
    }  
}

void Hex_Parsing(){
  if (0 == repeat) { 
    File file = LittleFS.open ("/testBinaryOutput.txt", "r");
    if (file.available()) {
      int counterByte=0;
      //packet[0] = 6479; //datalength 1 1 4 1 6468 4
      packet[0] = 6478 & 0xFF;       // Lower byte
      packet[1] = (6478 >> 8) & 0xFF; // Upper byte
      packet[2] = 0x16; //command
      //*(int*)(&packet[2]) = 0x08008000; //address
      // Store address as four bytes (assuming little-endian order)
      packet[3] = 0x00;
      packet[4] = 0x80;
      packet[5] = 0x00;
      packet[6] = 0x08;
      //packet[7] = 6468; //payload length
      // Store payload length 6468 as two bytes
      packet[7] = 6468 & 0xFF;       // Lower byte
      packet[8] = (6468 >> 8) & 0xFF; // Upper byte
      int dtlength = packet[7] + (packet[8] << 8);
      Serial.println (dtlength);
      while (counterByte < dtlength) {
        file.readBytes(buf,2);
        //Serial.printf("\nRead from file: %s", buf);
        char HexData = strtol(buf,NULL,16);
        packet[9+counterByte] = HexData;
        Serial.printf ("\npacket[%i] = %x", 9+counterByte,packet[9+counterByte]);
        counterByte++;
        delay (10);
      }
    Serial.println ("done hex parsing");
    repeat = 1; 
  }
 }
}

void JumptoAdd() {
  if ( 3 == repeat ) {
      uint8_t Jump_buffer[] = {0x14,0x00, 0x80, 0x00, 0x08,0xea,0x57,0x0d,0x7e}; //JUMP
      SPI.beginTransaction(mySettings);
      digitalWrite(SS_PIN, LOW);
      acknowledgment = SPI.transfer(0x09); //datalength
      Serial.print("Sent: ");
      Serial.println(0x09);
      if (acknowledgment == 0x03) {
        Serial.println("STM32 received the data successfully!");
        delay (5000);
        for (int i = 0; i <= 0x09 ; i++) {
          Serial.print("Ack =");
          Serial.println(acknowledgment);
          delay (100);
          digitalWrite(SS_PIN, HIGH);
          SPI.endTransaction();
          delay(1000);  // Adjust delay between transmissions
          SPI.beginTransaction(mySettings);
          digitalWrite(SS_PIN, LOW);
          acknowledgment = SPI.transfer(Jump_buffer[i]);
          Serial.println(Jump_buffer[i]);
        }
        repeat = 4;
      }
      else {
        Serial.print("Ack =");
        Serial.println(acknowledgment);
    
      }
        delay (100);
     
      // Deselect the Slave (SS) pin
      digitalWrite(SS_PIN, HIGH);
      
      // End the SPI transaction
      SPI.endTransaction();
    
      delay(10);  // Adjust delay between transmissions
  }
}
void SPIsend () {
 if (2 == repeat) {
  if (0x02 == acknowledgment) {
  SPI.beginTransaction(mySettings);
  digitalWrite(SS_PIN, LOW);
  acknowledgment = SPI.transfer(packet[0]); //datalength lower byte
  acknowledgment = SPI.transfer(packet[1]); //datalength higher byte
  Serial.print("Sent: ");
  Serial.println(packet[0],packet[1]);
  if (0x01 == acknowledgment) {
    Serial.println("STM32 received the data successfully!");
    delay (5000);
    for (int i = 1; i < packet[0]; i++) {
      Serial.print("Ack =");
      Serial.println(acknowledgment);
      delay (100);
      digitalWrite(SS_PIN, HIGH);
      SPI.endTransaction();
      delay(1000);  // Adjust delay between transmissions
      SPI.beginTransaction(mySettings);
      digitalWrite(SS_PIN, LOW);
      acknowledgment = SPI.transfer(packet[i]);
      Serial.println(packet[i]);
    }
  repeat = 3;
  }
  else {
    Serial.print("Ack =");
    Serial.println(acknowledgment);

  }
    delay (100);
 
  // Deselect the Slave (SS) pin
  digitalWrite(SS_PIN, HIGH);
  
  // End the SPI transaction
  SPI.endTransaction();

  delay(1000);  // Adjust delay between transmissions

 
 }
}
}
int one =1;
void erase_sector() {
  if ( 1 == repeat ) {
      uint8_t Erase_buffer[] = {0x15,0x02,0x01,0xea,0x57,0x0d,0x7e}; //ERASE
      SPI.beginTransaction(mySettings);
      digitalWrite(SS_PIN, LOW);
      acknowledgment = SPI.transfer(0x07); // Send the lower byte of the data length
      acknowledgment = SPI.transfer(0x00); // Send the higher byte of the data length (padding)
      Serial.print("Sent: ");
      Serial.println(0x07);
      one = 2;
      
      if (acknowledgment == 0x01) {
        Serial.println("STM32 received the data successfully!");
        delay (5000);
        for (int i = 0; i <= 0x07 ; i++) {
          Serial.print("Ack =");
          Serial.println(acknowledgment);
          delay (100);
          digitalWrite(SS_PIN, HIGH);
          SPI.endTransaction();
          delay(1000);  // Adjust delay between transmissions
          SPI.beginTransaction(mySettings);
          digitalWrite(SS_PIN, LOW);
          acknowledgment = SPI.transfer(Erase_buffer[i]);
          Serial.println(Erase_buffer[i]);
        }
        repeat = 2;
      }
      else {
        Serial.print("Ack =");
        Serial.println(acknowledgment);
    
      }
        delay (100);
     
      // Deselect the Slave (SS) pin
      digitalWrite(SS_PIN, HIGH);
      
      // End the SPI transaction
      SPI.endTransaction();
    
      delay(500);  // Adjust delay between transmissions
  }
}
