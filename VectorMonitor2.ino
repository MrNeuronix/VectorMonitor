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
const byte RX_BT = 12; // RX port
const byte TX_BT = 11; // TX port

// internal
byte LastPercent = 0;
const char date[20] = {};
const char wt[5] = {};
byte noData = 0;
byte pindex = 0;
byte data;
byte prevData;
byte packetData[250];
byte refresh = 0;
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
  if(refresh >= 120) {
    setupScreen();
    refresh = 0;
  }
  
  showDate();

  if (BTserial.overflow()) {
   Serial.println("SoftwareSerial overflow!");
   delay(400);
  }

  // read and skip all buffer
  while(BTserial.available()) {
    BTserial.read();
  }
  
  // sending TrmRequest (request controller for data)
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(113);    // command
  BTserial.write(-115);   // packet crc
  
  delay(300);

  while(BTserial.available()) {
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
        while(pindex < packetLength) {
          packetData[pindex] = BTserial.read();
          pindex = pindex + 1;
        }

        byte crc = BTserial.read();
      
        float speedCurrent = packetData[185] + (packetData[186] << 8);
        float batteryVoltage = ((float)(packetData[38] + (packetData[39] << 8)) / 10);
        float contTemp = ((float)(packetData[32] + (packetData[33] << 8))) / 10;
        float current = (((float)(packetData[20] + (packetData[21] << 8))) * 134) / 1000;
        float ah = ((((float)((long)packetData[83]) + (((long)(packetData[84])) << 8) + (((long)(packetData[85])) << 16) + (((long)(packetData[86])) << 24)) / 36000) * 134) / 1000;
        float ahRegen = ((((float)((long)packetData[87]) + (((long)(packetData[88])) << 8) + (((long)(packetData[89])) << 16) + (((long)(packetData[90])) << 24)) / 36000) * 134) / 1000;
        float distance = round(100 * (((((long)packetData[158]) + (((long)packetData[159]) << 8) + (((long)packetData[160]) << 16) + (((long)packetData[161]) << 24)) / ((float)99.9)) / 1000)) / 100;
        float odometer = round(100 * (((((long)packetData[26]) + (((long)packetData[27]) << 8) + (((long)packetData[28]) << 16) + (((long)packetData[29]) << 24)) / ((float)99.9)) / 1000)) / 100;

        Serial.print("Length: ");
        Serial.println(packetLength);
        Serial.print("CRC: ");
        Serial.println(crc);
        Serial.print("SPEED: ");
        Serial.println(abs(speedCurrent));
        Serial.print("VOLTAGE: ");
        Serial.println(batteryVoltage);
        Serial.print("AH: ");
        Serial.println(ah);
        Serial.print("AHR: ");
        Serial.println(ahRegen);
        Serial.print("CTemp: ");
        Serial.println(contTemp);

        if(speedCurrent == -1) {
          speedCurrent = 0;
        }
  
        showSpeed(speedCurrent);
        showPower(abs(batteryVoltage * current));
        drawBar(abs(ah - ahRegen) / (BATTERY_CAPACITY / 100));

        // read and skip all buffer
        while(BTserial.available()) {
          BTserial.read();
        }

        delay(100);
      }
      else {
        Serial.println("Unknown packet type");
      }
    }
        
    prevData = data;
  }

  if(noData > 10 && initialized) {
   setupNoDataScreen();
   initialized = false;
  }
    
  if(initialized) {
    noData = noData + 1;
  }

  delay(100);
  
  if(timeStatus() != timeSet) {
    Serial.println("RTC ERROR: SYNC!");
  }
}

void setupScreen(void) {
    tft.fillScreen(BLACK);
  
    LastPercent = 0;
  
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

void showSpeed(const int kmh) {
  tft.fillRect(30, 10, 140, 130,  BLACK);
  showmsgXY(30, 120, 2, GREEN, &FreeSevenSegNumFont, String(kmh));
}

void showPower(const float watt) {
  sprintf(wt, "%04d", (int)watt);  
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
  nPer = 100-nPer;
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
 *  packetData[20] // Curr1
 *  packetData[21] // Curr1
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
 *  packetData[34] // Temperature1
 *  packetData[35] // Temperature1
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
