// Vector-S/M Monitor v3
// Special for ElectroTrasport.Ru
// (c) 2019 Nikolay Viguro aka Neuronix

#include "Nextion.h"
#include <TimeLib.h>
#include <DS1302RTC.h>
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <ArduinoOTA.h>

const char *ssid = "Vortex";
const char *password = "monitormonitor"; // must be greater than 8 chars

const int SerialBaudRate = 115200; 
const int BtBaudRate = 9600; //38400;

const float BATTERY_CAPACITY = 60; // in Ah

// RTC pins
const byte RST_PIN = 13;
const byte DAT_PIN = 10;
const byte CLK_PIN = 19; // A5;  

// HC-05 bluetooth adapter settings
const byte RX_BT = 14; // RX port
const byte TX_BT = 27; // TX port

// internal
Preferences cfg;

byte noData = 0;
byte pindex = 0;
int data;
int prevData;
int packetData[255];
unsigned int voltageInROM;

const char VERSION[10] = "v.1.0.1";

struct CurrentData 
{
 float speed;
 float voltage;
 float current;
 float ah;
 float ahRegen;
 float tempCont;
 float tempEngine;
 float distance;
 float odometer;
};
typedef CurrentData CurrentData;

struct Label 
{
 NexText speed = NexText(1, 3, "speed");
 NexText voltage = NexText(1, 5, "volts");
 NexText current = NexText(1, 6, "current");
 NexText ah = NexText(1, 13, "ah");
 NexText ahRegen = NexText(1, 15, "ahRegen");
 NexText tempCont = NexText(1, 20, "controllerTemp");
 NexText tempEngine = NexText(1, 18, "engineTemp");
 NexText distance = NexText(1, 21, "distance");
 NexText odometer = NexText(1, 22, "odometer");
 NexText power = NexText(1, 4, "watts");
 NexProgressBar ahBar = NexProgressBar(1, 1, "ahBar");
 NexText date = NexText(1, 11, "date");
 NexText time = NexText(1, 12, "time");
 
 NexPage splash = NexPage(0, 0, "splash");
 NexPage main = NexPage(1, 0, "main");

 NexText status = NexText(0, 2, "status");
 NexText version = NexText(0, 3, "version");
};
typedef Label Label;

boolean initialized = false;
boolean firstRun = true;
boolean regen = false;
boolean controllerLocked = true;

DS1302RTC RTC(RST_PIN, DAT_PIN, CLK_PIN);
HardwareSerial BTserial(1);
AsyncWebServer server(80);

CurrentData currentData;
Label labels;

void setup(void)
{
    Serial.begin(SerialBaudRate);
    BTserial.begin(BtBaudRate, SERIAL_8N1, RX_BT, TX_BT);
    BTserial.setRxBufferSize(1024);
    cfg.begin("vortex", false);

    nexInit();
    labels.splash.show();
    labels.version.setText(VERSION);
    labels.status.setText("Starting up WiFi AP...");

    const IPAddress apIP = IPAddress(192, 168, 1, 1);
    
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    ArduinoOTA.setHostname("Vortex");
    ArduinoOTA.setPort(3232);

    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
  
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
  
    ArduinoOTA.begin();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Hello, world");
    });

    voltageInROM = cfg.getUInt("voltage", 0);
    
    // set it for first launch
    //RTC.writeEN(true);
    //setTime(21, 01, 00, 26, 2, 2019); // hh:mm:ss dd:mm:yyyy
    //RTC.set(now());

    labels.status.setText("Setting up clock...");

    if (RTC.haltRTC()) {
      Serial.println(F("Clock stopped"));
    } else {
      Serial.println(F("Clock working!"));
    }
    
    if (RTC.writeEN()) {
      Serial.println(F("Write allowed."));
    } else {
      Serial.println(F("Write protected"));
    }

    setSyncProvider(RTC.get); // the function to get the time from the RTC
    if(timeStatus() == timeSet) {
      Serial.println(F(" Sync Ok! "));
    } else {
      Serial.println(F("SYNC FAIL!"));
    }

    server.onNotFound(notFound);
    server.begin();

    labels.status.setText("Reseting trip...");

    sendResetTrip();

    labels.status.setText("Waiting for data packet...");
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void loop(void) { 
  ArduinoOTA.handle();
   
  showDate();

  delay(200);
  
  // sending TrmRequest (request controller for data)
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(113);    // command
  BTserial.write(-115);   // packet crc

  while(BTserial.available()) {
    data = BTserial.read();
  
    if(data == 0xFF && prevData == 0xFF) {
      byte packetLength = BTserial.read();
      byte packetType = BTserial.read();
  
      // data packet (5)
      if(packetType == 5) {
        memset(packetData, 0, sizeof(packetData));
        pindex = 0;
    
        // reading response data packet
        // packet structure described below code
        
        // we are waiting for all data comes to RX
        while(BTserial.available() < packetLength-1) {
          delay(5);
        }

        while(pindex < packetLength-1) { // length = type byte + dataBytes[n]
          packetData[pindex] = BTserial.read();
          pindex++;
        }

        int crc = BTserial.read();
        int calcCrc = calculateCRC(packetType, packetLength, packetData, crc);

        if(crc != calcCrc) {
          Serial.println(F("Bad packet"));
          noData++;
          continue;
        }

        if(!initialized) {
          if(firstRun) {
            firstRun = false; // do this check only once at startup
            byte batteryVoltage = ((float)(packetData[38] + (packetData[39] << 8)) / 10);

            Serial.print(F("Current voltage is: "));
            Serial.println(batteryVoltage);
            Serial.print(F("Remembered voltage is: "));
            Serial.println(voltageInROM);
  
            // if previous remembered battery voltage lower than current value
            // we suppose that the battery was recharged
            // So we reset Ah counters and store new value in EEPROM
            if(voltageInROM < batteryVoltage) {
              Serial.println(F("Reseting Ah counters"));
              labels.status.setText("Reseting Ah counters...");
              
              delay(500);
              sendResetAh();
            }

              voltageInROM = batteryVoltage;
              cfg.putUInt("voltage", voltageInROM);
              cfg.end();
          }
          
          setupScreen();
          initialized = true;
        }
    
        if(noData > 0) {
          noData = 0;
        }
      
        currentData.speed = packetData[185] + (packetData[186] << 8);
        currentData.voltage = ((float)(packetData[38] + (packetData[39] << 8)) / 10);
        currentData.tempCont = ((float)(packetData[32] + (packetData[33] << 8))) / 10;
        currentData.tempEngine = ((float)(packetData[34] + (packetData[35] << 8))) / 10;
        currentData.ah = (float)(((((float)((long)packetData[83]) + (((long)(packetData[84])) << 8) + (((long)(packetData[85])) << 16) + (((long)(packetData[86])) << 24)) / 36000) * 134) / 1000);
        currentData.ahRegen = (float)(((((float)((long)packetData[87]) + (((long)(packetData[88])) << 8) + (((long)(packetData[89])) << 16) + (((long)(packetData[90])) << 24)) / 36000) * 134) / 1000);
        currentData.distance = (float)(round(100 * (((((long)packetData[158]) + (((long)packetData[159]) << 8) + (((long)packetData[160]) << 16) + (((long)packetData[161]) << 24)) / ((float)99.9)) / 1000)) / 100);
        currentData.odometer = (float)(round(100 * (((((long)packetData[26]) + (((long)packetData[27]) << 8) + (((long)packetData[28]) << 16) + (((long)packetData[29]) << 24)) / ((float)99.9)) / 1000)) / 100);

        // FIXME! skip negative current (problem with second byte)
        if((packetData[21] << 8) > 64000) {
          currentData.current = 0.0;
          regen = true;
        } else {
          currentData.current = ((float)((packetData[20] + (packetData[21] << 8))) * ((float)134)) / ((float)1000);
        }

        static char speed[1];
        dtostrf(currentData.speed, 2, 0, speed);
        labels.speed.setText(speed);

        static char voltage[4];
        dtostrf(currentData.voltage, 5, 2, voltage);
        labels.voltage.setText(voltage);
        
        static char current[5];

        if(regen) {// FIXME! regen
          labels.current.setText("regen");
        }
        else {
          regen = false;
          
          if(currentData.current > 100) {
            dtostrf(currentData.current, 6, 0, current);
          } else {
            dtostrf(currentData.current, 6, 2, current);
          }
          labels.current.setText(current);
        }

        static char tempCont[5];
        dtostrf(currentData.tempCont, 6, 1, tempCont);
        labels.tempCont.setText(tempCont);

        static char tempEngine[5];
        dtostrf(currentData.tempEngine, 6, 1, tempEngine);
        labels.tempEngine.setText(tempEngine);

        static char ah[4];
        dtostrf(fabs(currentData.ah), 5, 1, ah);
        labels.ah.setText(ah);

        static char ahRegen[4];
        dtostrf(fabs(currentData.ahRegen), 5, 1, ahRegen);
        labels.ahRegen.setText(ahRegen);

        static char distance[6];
        static char distanceKm[9];
        dtostrf(currentData.distance, 7, 2, distance);
        sprintf(distanceKm,"%s km", distance);
        labels.distance.setText(distanceKm);

        static char odometer[6];
        static char odometerKm[9];
        dtostrf(currentData.odometer, 7, 0, odometer);
        sprintf(odometerKm,"%s km", odometer);
        labels.odometer.setText(odometerKm);

        if(!regen) {
          static char power[4];
          dtostrf(currentData.voltage * currentData.current, 5, 0, power);
          labels.power.setText(power);
        } else {
          labels.power.setText("----");
        }

        labels.ahBar.setValue(100 - (fabs(ah - ahRegen) / (BATTERY_CAPACITY / 100)));
      }
    }
        
    prevData = data;
  }

  if(noData > 10 && initialized) {
   setupNoDataScreen();
   labels.status.setText("Waiting for data from controller...");
   initialized = false;
  }
    
  if(initialized) {
    noData++;
  }
}

byte calculateCRC(int type, int plength, int *data, int crc) {
  unsigned int summ = type + plength;
  
  for (byte j = 0; j < (plength-1); j++) {
    summ = summ + data[j];
  }
  summ = ~summ;
  
  return (byte)summ;
}

void sendUnlock(void) {
  // send SEND_unlock command for unlock controller
  BTserial.write(-1);      // 0xff
  BTserial.write(-1);      // 0xff
  BTserial.write(13);      // data length
  BTserial.write(243);     // command
  BTserial.write(0x37);    // data
  BTserial.write(0xac);    // data
  BTserial.write(0x2b);    // data
  BTserial.write(0x33);    // data
  BTserial.write(0xf1);    // data
  BTserial.write(0x91);    // data
  BTserial.write(0x7a);    // data
  BTserial.write(0xb0);    // data
  BTserial.write(0xec);    // data
  BTserial.write(0x46);    // data
  BTserial.write(0x10);    // data
  BTserial.write(0xaa);    // data
  BTserial.write(38);      // packet crc 
}

void sendLock(void) {
  // send SEND_lock command for lock again
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(245);    // command
  BTserial.write(9);      // packet crc  
}

void sendSaveSettings(void) {
  // sending SEND_ProgrammOptions
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(15);     // command
  BTserial.write(-17);    // packet crc
}

void sendResetTrip(void) {
  sendUnlock();

  delay(200);
  
  // sending SEND_ResetDistance
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(184);    // command
  BTserial.write(70);     // packet crc

  delay(200);

  sendSaveSettings();

  delay(300);

  sendLock();
}

void sendResetAh(void) {
  sendUnlock();

  delay(200);

  // send SEND_ClearCurrentAH command
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(108);    // command
  BTserial.write(-110);   // packet crc

  delay(200);

  sendSaveSettings();

  delay(300);

  sendLock();
}

void sendStartCharge(void) {
  sendUnlock();

  delay(200);

  // send SEND_ChagerViaMotorOn command
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(2);      // data length
  BTserial.write(254);    // NEW_CMD byte
  BTserial.write(66);     // SEND_ChagerViaMotorOn
  BTserial.write(-68);    // packet crc

  delay(200);

  sendLock();
}

void sendStopCharge(void) {
  sendUnlock();

  delay(200);

  // send SEND_ChagerViaMotorOff command
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(2);      // data length
  BTserial.write(254);    // NEW_CMD byte
  BTserial.write(67);     // SEND_ChagerViaMotorOff
  BTserial.write(-69);    // packet crc

  delay(200);

  sendLock();
}

void showDate() {
  static char dataLabel[10];  
  
  sprintf(dataLabel, "%02d:%02d", hour(), minute());
  labels.time.setText(dataLabel);
  
  sprintf(dataLabel, "%02d.%02d.%04d", day(), month(), year());
  labels.date.setText(dataLabel);
  
}

void setupScreen(void) {
    labels.main.show();
}

void setupNoDataScreen(void) {
    labels.splash.show();
    labels.version.setText(VERSION);
}

/* 
 *  -----------------------------------------------
 *  Data (5) packet structure
 *  -----------------------------------------------
 *  
 *  packetData[0] // CntSample
 *  packetData[1] // CntSample
 *  packetData[2] // CntSample
 *  packetData[3] // CntSample
 *  
 *  packetData[4] // AlfaXRes
 *  packetData[5] // AlfaXRes
 *  
 *  packetData[6] // AlfaYRes
 *  packetData[7] // AlfaYRes
 *  
 *  packetData[8] // GyroX
 *  packetData[9] // GyroX
 *  
 *  packetData[10] // TiltXres
 *  packetData[11] // TiltXres
 *  
 *  packetData[12] // TiltYres
 *  packetData[13] // TiltYres
 *  
 *  packetData[14] // Ep
 *  packetData[15] // Ep
 *  
 *  packetData[16] // Ei
 *  packetData[17] // Ei
 *  
 *  packetData[18] // PWM1
 *  packetData[19] // PWM1
 *  
 *  packetData[20] // Curr1 --- battery current
 *  packetData[21] // Curr1 --- battery current
 *  
 *  packetData[22] // Spd1USTKMH
 *  packetData[23] // Spd1USTKMH
 *  
 *  packetData[24] // Temperature3
 *  packetData[25] // Temperature3
 *  
 *  packetData[26] // Odometer
 *  packetData[27] // Odometer
 *  packetData[28] // Odometer
 *  packetData[29] // Odometer
 *  
 *  packetData[30] // Spd1Res
 *  packetData[31] // Spd1Res
 *  
 *  packetData[32] // Temperature2 --- controller temp
 *  packetData[33] // Temperature2 --- controller temp
 *  
 *  packetData[34] // Temperature1 --- engine temp
 *  packetData[35] // Temperature1 --- engine temp
 *  
 *  packetData[36] // Kp
 *  packetData[37] // Kp
 *  
 *  packetData[38] // UBT ------ BATTERY
 *  packetData[39] // UBT ------ VOLTAGE
 *  
 *  packetData[40] // GyroVert_Steer
 *  packetData[41] // GyroVert_Steer
 *  
 *  packetData[42] // StatFlags
 *  packetData[43] // StatFlags
 *  
 *  packetData[44] // GyroZFilter
 *  packetData[45] // GyroZFilter
 *  
 *  packetData[46] // AlfaYResPrevAv
 *  packetData[47] // AlfaYResPrevAv
 *  
 *  packetData[48] // DiffBetweenTilts
 *  packetData[49] // DiffBetweenTilts
 *  
 *  packetData[50] // Flgs
 *  packetData[51] // Flgs
 *  packetData[52] // Flgs
 *  packetData[53] // Flgs
 *  
 *  packetData[54] // Flgs1
 *  packetData[55] // Flgs1
 *  packetData[56] // Flgs1
 *  packetData[57] // Flgs1
 *  
 *  packetData[58] // Flgs2
 *  packetData[59] // Flgs2
 *  packetData[60] // Flgs2
 *  packetData[61] // Flgs2
 *  
 *  packetData[62] // TiltZad
 *  packetData[63] // TiltZad
 *  
 *  packetData[64] // StartRot
 *  packetData[65] // StartRot
 *  
 *  packetData[66] // Ki
 *  packetData[67] // Ki
 *  
 *  packetData[68] // KRot
 *  packetData[69] // KRot
 *  
 *  packetData[70] // KpRot
 *  packetData[71] // KpRot
 *  
 *  packetData[72] // KiRot
 *  packetData[73] // KiRot
 *  
 *  packetData[74] // Preas1ADC
 *  packetData[75] // Preas1ADC
 *  
 *  packetData[76] // Preas2ADC
 *  packetData[77] // Preas2ADC
 *  
 *  packetData[78] // CurrLimitTek
 *  
 *  packetData[79] // KpSPD
 *  packetData[80] // KpSPD
 *  
 *  packetData[81] // AngleLimitReal
 *  packetData[82] // AngleLimitReal
 *  
 *  packetData[83] // CurrTuda1 ------------
 *  packetData[84] // CurrTuda1 ------------  Ah consumed
 *  packetData[85] // CurrTuda1 ------------
 *  packetData[86] // CurrTuda1 ------------
 *  
 *  packetData[87] // CurrRegen1 ------------
 *  packetData[88] // CurrRegen1 ------------ Ah regen
 *  packetData[89] // CurrRegen1 ------------
 *  packetData[90] // CurrRegen1 ------------
 *  
 *  packetData[91] // Flgs4
 *  packetData[92] // Flgs4
 *  packetData[93] // Flgs4
 *  packetData[94] // Flgs4
 *  
 *  packetData[95] // Flgs5
 *  packetData[96] // Flgs5
 *  packetData[97] // Flgs5
 *  packetData[98] // Flgs5
 *  
 *  packetData[99] // Flgs3
 *  packetData[100] // Flgs3
 *  packetData[101] // Flgs3
 *  packetData[102] // Flgs3
 *  
 *  packetData[103] // TimerCnt
 *  packetData[104] // TimerCnt
 *  packetData[105] // TimerCnt
 *  packetData[106] // TimerCnt
 *  
 *  packetData[107] // _3VFl
 *  packetData[108] // _3VFl
 *  
 *  packetData[109] // _5VFl
 *  packetData[110] // _5VFl
 *  
 *  packetData[111] // _12VFl
 *  packetData[112] // _12VFl
 *  
 *  packetData[113] // V4
 *  packetData[114] // V4
 *  
 *  packetData[115] // V5
 *  packetData[116] // V5
 *  
 *  packetData[117] // V6
 *  packetData[118] // V6
 *  
 *  packetData[119] // V7
 *  packetData[120] // V7
 *  
 *  packetData[121] // V8
 *  packetData[122] // V8
 *  
 *  packetData[123] // V9
 *  packetData[124] // V9
 *  
 *  packetData[125] // V10
 *  packetData[126] // V10
 *  
 *  packetData[127] // V11
 *  packetData[128] // V11 
 *  
 *  packetData[129] // V12
 *  packetData[130] // V12
 *  
 *  packetData[131] // V13
 *  packetData[132] // V13
 *  
 *  packetData[133] // V14
 *  packetData[134] // V14
 *  
 *  packetData[135] // V15
 *  packetData[136] // V15
 *  
 *  packetData[137] // V16
 *  packetData[138] // V16
 *  
 *  packetData[139] // V17
 *  packetData[140] // V17
 *  
 *  packetData[141] // V18
 *  packetData[142] // V18
 *  
 *  packetData[143] // BatteryKeys
 *  packetData[144] // BatteryKeys
 *  packetData[145] // BatteryKeys
 *  packetData[146] // BatteryKeys
 *  
 *  packetData[147] // ChagerKeys
 *  
 *  packetData[148] // AccX
 *  packetData[149] // AccX
 *  
 *  packetData[150] // AccY
 *  packetData[151] // AccY
 *  
 *  packetData[152] // AccZ
 *  packetData[153] // AccZ
 *  
 *  packetData[154] // Sensor1_Prev
 *  
 *  packetData[155] // Sensor2_Prev
 *  
 *  packetData[156] // Temperature4
 *  packetData[157] // Temperature4
 *  
 *  packetData[158] // Distance
 *  packetData[159] // Distance
 *  packetData[160] // Distance
 *  packetData[161] // Distance
 *  
 *  packetData[162] // ProtectValue
 *  packetData[163] // ProtectValue
 *  
 *  packetData[164] // Temperature
 *  
 *  packetData[165] // _48V
 *  
 *  packetData[166] // KAccGyroSteer
 *  
 *  packetData[167] // GyroXFl
 *  packetData[168] // GyroXFl
 *  
 *  packetData[169] // GyroYFl
 *  packetData[170] // GyroYFl
 *  
 *  packetData[171] // GyroZFl
 *  packetData[172] // GyroZFl
 *  
 *  packetData[173] // Ed
 *  packetData[174] // Ed
 *  
 *  packetData[175] // GyroYAv
 *  packetData[176] // GyroYAv
 *  
 *  packetData[177] // KdI
 *  packetData[178] // KdI
 *  
 *  packetData[179] // MaxSpdKMH
 *  packetData[180] // MaxSpdKMH
 *  
 *  packetData[181] // PhasesCnt2
 *  packetData[182] // PhasesCnt2
 *  packetData[183] // PhasesCnt2
 *  packetData[184] // PhasesCnt2
 *  
 *  packetData[185] // Spd1Fl --\ SPEED
 *  packetData[186] // Spd1Fl --/ SPEED
 *  
 *  packetData[187] // Temperature5
 *  packetData[188] // Temperature5 
 *  
 *  packetData[189] // Temperature6
 *  packetData[190] // Temperature6 
 *  
 *  packetData[191] // Phase1Period1
 *  packetData[192] // Phase1Period1 
 *  packetData[193] // Phase1Period1
 *  packetData[194] // Phase1Period1 
 *  
 *  packetData[195] // Phase1Period2
 *  packetData[196] // Phase1Period2 
 *  packetData[197] // Phase1Period2
 *  packetData[198] // Phase1Period2 
 *
 *  packetData[199] // EpCurr1
 *  packetData[200] // EpCurr1 
 *  packetData[201] // EpCurr1
 *  packetData[202] // EpCurr1 
 *  
 *  packetData[203] // Phase1Period4
 *  packetData[204] // Phase1Period4 
 *  packetData[205] // Phase1Period4
 *  packetData[206] // Phase1Period4 
 *  
 *  packetData[207] // Phase1Period5
 *  packetData[208] // Phase1Period5 
 *  packetData[209] // Phase1Period5
 *  packetData[210] // Phase1Period5 
 *  
 *  packetData[211] // Phase1Period6
 *  packetData[212] // Phase1Period6  
 *  packetData[213] // Phase1Period6
 *  packetData[214] // Phase1Period6 
 *  
 *  packetData[215] // PDC3
 *  packetData[216] // PDC3
 *  
 *  packetData[217] // CriticalError
 *  
 *  packetData[218] // Preas1ADCResCalibr
 *  packetData[219] // Preas1ADCResCalibr 
 *  
 *  packetData[220] // ADCThrottleBreak
 *  packetData[221] // ADCThrottleBreak 
 *  
 *  packetData[222] // RCON_Mem
 *  packetData[223] // RCON_Mem 
 *  
 *  packetData[224] // FirmwareVersion
 *  packetData[225] // FirmwareVersion 
 *  
 *  packetData[226] // MPU6050Err
 *  
 *  packetData[227] // PhaseCurr
 *  packetData[228] // PhaseCurr 
 *  
 *  packetData[229] // HallErrCnt
 *  packetData[230] // HallErrCnt 
 *  
 *  packetData[231] // CurrPhaseLimitTek
 *  packetData[232] // CurrPhaseLimitTek
 *  
 *  packetData[233] // PhasePer1
 *  packetData[234] // PhasePer1 
 *  packetData[235] // PhasePer1
 *  packetData[236] // PhasePer1 
 *  
 *  packetData[237] // Halls
 *  
 *  packetData[238] // HallDelay1_Tek_F
 *  
 *  packetData[239] // HallDelay1_Tek_B 
 */
