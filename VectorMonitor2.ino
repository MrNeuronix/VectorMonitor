// Vector-S/M Monitor v2 for 3.5" TFT LCD
// Special for ElectroTrasport.Ru
// (c) 2018 Nikolay Viguro aka Neuronix

#include <Adafruit_GFX.h>    // Core graphics library
#include <MCUFRIEND_kbv.h>   // Hardware-specific library
#include <DS1302RTC.h>
#include <SoftwareSerial.h>
#include <FreeDefaultFonts.h>

#define BLACK   0x0000
#define RED     0xF800
#define GREEN   0x07E0
#define WHITE   0xFFFF
#define GREY    0x8410

const int SerialBaudRate = 9600; 
const int BtBaudRate = 9600; //38400;

const float BATTERY_CAPACITY = 60; // in Ah

// RTC pins
const byte RST_PIN = 13;
const byte DAT_PIN = 10;
const byte CLK_PIN = 19; // A5;  

// HC-05 bluetooth adapter settings
const byte TX_BT = 12; // TX port
const byte RX_BT = 11; // RX port

// internal
byte LastPercent = 0;
const char date[20] = {};
const char wt[5] = {};
byte noData = 0;
byte pindex = 0;
byte data;
byte prevData;
byte packetData[255];
boolean initialized = false;

MCUFRIEND_kbv tft;
DS1302RTC RTC(RST_PIN, DAT_PIN, CLK_PIN);
SoftwareSerial BTserial(RX_BT, TX_BT); // you *MUST* increase byte buffer in SoftwareSerial.h to 255 for large data packet from controller

void setup(void)
{
    Serial.begin(SerialBaudRate);
    BTserial.begin(BtBaudRate);
    
    uint16_t ID = tft.readID();
    if (ID == 0xD3) ID = 0x9481;
    
    tft.begin(ID);
    tft.setRotation(1);
    
    tft.fillScreen(BLACK);

    // set it for first launch
    //RTC.writeEN(true);
    //setTime(22, 38, 00, 19, 2, 2019); // hh:mm:ss dd:mm:yyyy
    //RTC.set(now());

    if (RTC.haltRTC()) {
      Serial.println("Clock stopped");
    } else {
      Serial.println("Clock working!");
    }
    
    if (RTC.writeEN()) {
      Serial.println("Write allowed.");
    } else {
      Serial.println("Write protected");
    }

    delay ( 1000 );

    setSyncProvider(RTC.get); // the function to get the time from the RTC
    if(timeStatus() == timeSet) {
      Serial.println(" Sync Ok! ");
    } else {
      Serial.println("SYNC FAIL!");
    }

    // interface
    setupNoDataScreen();
}

void loop(void) {
  showDate();
  
  // sending TrmRequest (request controller for data)
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(113);    // command
  BTserial.write(-115);   // packet crc
  
  delay(500);

  if(BTserial.available()) {
    data = BTserial.read();
  
    if(data == 0xFF && prevData == 0xFF) {
      // new packet detected
      Serial.println("New packet detected");
      byte packetLength = BTserial.read();
      byte packetType = BTserial.read();
  
      // data packet (5)
      if(packetType == 5) {
        Serial.println("New data packet captured");
        memset(packetData, 0, sizeof(packetData));
        pindex = 0;
  
        if(!initialized) {
          setupScreen();
          initialized = true;
        }
    
        if(noData > 0) {
          noData = 0;
        }
    
        // reading response data packet
        // packet structure described below code
        while(pindex <= packetLength-1) {
          packetData[pindex] = BTserial.read();
          pindex++;
        }
      
        float speedCurrent = packetData[185] + (packetData[186] << 8);
        float batteryVoltage = ((float)(packetData[38] + (packetData[39] << 8)) / 10);
        float contTemp = ((float)(packetData[32] + (packetData[33] << 8))) / 10;
        float current = (((float)(packetData[20] + (packetData[21] << 8))) * 134) / 1000;
        float ah = ((((float)((long)packetData[83]) + (((long)(packetData[84])) << 8) + (((long)(packetData[85])) << 16) + (((long)(packetData[86])) << 24)) / 36000) * 134) / 1000;
        float ahRegen = ((((float)((long)packetData[87]) + (((long)(packetData[88])) << 8) + (((long)(packetData[89])) << 16) + (((long)(packetData[90])) << 24)) / 36000) * 134) / 1000;
        float distance = round(100 * (((((long)packetData[158]) + (((long)packetData[159]) << 8) + (((long)packetData[160]) << 16) + (((long)packetData[161]) << 24)) / ((float)99.9)) / 1000)) / 100;
        float odometer = round(100 * (((((long)packetData[26]) + (((long)packetData[27]) << 8) + (((long)packetData[28]) << 16) + (((long)packetData[29]) << 24)) / ((float)99.9)) / 1000)) / 100;
  
        showSpeed(speedCurrent);
        showPower(batteryVoltage * current);
        drawBar(abs(ah - ahRegen) / (BATTERY_CAPACITY / 100));
      }
    }
    
    prevData = data;
  } else {
    if(noData > 10 && initialized) {
      setupNoDataScreen();
      initialized = false;
    }

    if(initialized) {
      noData = noData + 1;
    }
  }
  
  // Warning!
  if(timeStatus() != timeSet) {
    Serial.println("RTC ERROR: SYNC!");
  }
}

void setupScreen(void) {
    tft.fillScreen(BLACK);
  
    drawScale();

    // watt label
    tft.setCursor(410, 50);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.print("watt");

    // kmh
    tft.setCursor(175, 30);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.print("km/h");
}

void setupNoDataScreen(void) {
    tft.fillScreen(BLACK);

    tft.setCursor(170, 130);
    tft.setTextSize(4);
    tft.setTextColor(GREEN);
    tft.print("Vortex");

    tft.setCursor(100, 180);
    tft.setTextSize(3);
    tft.setTextColor(WHITE);
    tft.print("Waiting for data");
}

char * float2s(float f) {
  char result[] = "";
  sprintf(result, "%.2f", f);
  return result;
}

void showDate() {
  sprintf(date, "%02d-%02d-%04d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  showmsgXY(355, 15, 1, WHITE, NULL, String(date));
}

void showSpeed(const float kmh) {
  Serial.print("SPEED: ");
  Serial.println(kmh);

  tft.fillRect(30, 10, 140, 130,  BLACK);
  
  showmsgXY(30, 120, 2, GREEN, &FreeSevenSegNumFont, String(kmh));
}

void showPower(const float watt) {
  sprintf(wt, "%04d", (int)watt);

  Serial.print("WATT: ");
  Serial.println(wt);
  
  showmsgXY(280, 50, 5, GREY, NULL, String(wt));
}

void drawScale(){  
   tft.drawFastVLine(65, 150,100, WHITE); // Vertical Scale Line 
  
   tft.setCursor(15, 150);
   tft.setTextSize(1);
   tft.setTextColor(WHITE);
   tft.print(BATTERY_CAPACITY);
   tft.drawFastHLine(57, 150, 8, WHITE); // Major Division

   tft.setCursor(15, 170);
   tft.setTextSize(1);
   tft.setTextColor(WHITE);
   tft.print((BATTERY_CAPACITY / 100) * 75);
   tft.drawFastHLine(60, 174, 5, WHITE); // Minor Division

   tft.setCursor(15, 195);
   tft.setTextSize(1);
   tft.setTextColor(WHITE);
   tft.print((BATTERY_CAPACITY / 100) * 50);
   tft.drawFastHLine(57, 199, 8, WHITE); // Major Division

   tft.setCursor(15, 220);
   tft.setTextSize(1);
   tft.setTextColor(WHITE);
   tft.print((BATTERY_CAPACITY / 100) * 25);
   tft.drawFastHLine(60, 224, 5, WHITE); // Minor Division

   tft.setCursor(15, 244);
   tft.setTextSize(1);
   tft.setTextColor(WHITE);
   tft.print(0);
   tft.drawFastHLine(57, 249, 8, WHITE);  // Major Division
}

void drawBar (int nPer) {
  if(nPer < LastPercent){
    tft.fillRect(77, 150 + (100-LastPercent), 30, LastPercent - nPer,  BLACK);     
  }
  else{
    tft.fillRect(77, 150 + (100-nPer), 30, nPer - LastPercent,  GREEN);
  }    
  LastPercent = nPer;  
}

void showmsgXY(int x, int y, int sz, int color, const GFXfont *f, String msg) {
     tft.setFont(f);
     tft.setCursor(x, y);
     tft.setTextSize(sz);
     tft.setTextColor(color, BLACK);
     tft.print(msg);
}
