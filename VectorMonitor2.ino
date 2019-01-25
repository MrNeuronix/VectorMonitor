// Vector-S/M Monitor
// Special for ElectroTrasport.Ru
// (c) 2018 Nikolay Viguro aka Neuronix

#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <math.h>
 
SoftwareSerial BTserial(D3, D4, false, 255); // RX, TX
LiquidCrystal_I2C lcd(0x27, 20, 4); 

const long SerialBaudRate = 9600; 
const long BtBaudRate = 9600; //38400; 

const float BATTERY_AH = 20;

byte pindex = 0;
byte data;
byte prevData;
byte packetData[255];

void setup() {
  Serial.begin(SerialBaudRate);  
  BTserial.begin(BtBaudRate);  
  Wire.begin(D6, D7);

  lcd.init();
  lcd.backlight();                     
  lcd.home();

  lcd.setCursor(2, 1);
  lcd.print("Vector-M Monitor");
  lcd.setCursor(5, 2);
  lcd.print("ver. 0.1");

  delay(2500);

  lcd.clear();

  // voltage
  lcd.setCursor(2, 0);
  lcd.print("00.00");
  lcd.setCursor(7, 0);
  lcd.print("V");

  // speed
  lcd.setCursor(10, 0);
  lcd.print("000.00");
  lcd.setCursor(17, 0);
  lcd.print("kmh");

  // power W
  lcd.setCursor(0, 1);
  lcd.print("0000.00");
  lcd.setCursor(7, 1);
  lcd.print("W");

  // power A
  lcd.setCursor(10, 1);
  lcd.print("000.00");
  lcd.setCursor(17, 1);
  lcd.print("A");

  // power consumed
  lcd.setCursor(0, 2);
  lcd.print("C:");
  lcd.setCursor(2, 2);
  lcd.print("00.00");
  lcd.setCursor(7, 2);
  lcd.print("Ah");

  // power regen
  lcd.setCursor(11, 2);
  lcd.print("00.00");
  lcd.setCursor(17, 2);
  lcd.print("Ah");

  // temp
  lcd.setCursor(0, 3);
  lcd.print("T:");
  lcd.setCursor(2, 3);
  lcd.print("000.00");
  lcd.setCursor(8, 3);
  lcd.print("C");

  // trip
  lcd.setCursor(10, 3);
  lcd.print("000.00");
  lcd.setCursor(17, 3);
  lcd.print("km");
}
  
void loop() {

  // sending TrmRequest (request controller for data)
  BTserial.write(-1);     // 0xff
  BTserial.write(-1);     // 0xff
  BTserial.write(1);      // data length
  BTserial.write(113);    // command
  BTserial.write(-115);   // packet crc
  
  delay(500);
  
  BTserial.read(); // 0xff
  BTserial.read(); // 0xff
    
  byte packetLength = BTserial.read();
  byte packetType = BTserial.read();
  
  if(packetType == 5) {
    memset(packetData, 0, sizeof(packetData));
    pindex = 0;
    
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

    // battery level icon
    batterylevel(0, 0, abs(ah + ahRegen));

    // battegy voltage
    lcd.setCursor(2, 0);
    lcd.print(float2s(batteryVoltage));

    // speed
    lcd.setCursor(10, 0);
    if(speedCurrent < 100) {
      if(speedCurrent < 10) {
        lcd.print("  ");
        lcd.print(float2s(speedCurrent));
      } else {
        lcd.print(" ");
        lcd.print(float2s(speedCurrent));
      }
    } else {
        lcd.print(float2s(speedCurrent));
    }

    // power W
    lcd.setCursor(0, 1);
    float power = batteryVoltage * current;
    if(power < 1000) {
      if(power < 100) {
        if(power < 10) {
          lcd.print("   ");
          lcd.print(float2s(power));
        } else {
          lcd.print("  ");
          lcd.print(float2s(power));
        }
      } else {
          lcd.print(" ");
          lcd.print(float2s(power));
      }
    } else {
      lcd.print(float2s(power));
    }

    // power A
    lcd.setCursor(10, 1);
    if(current < 100) {
      if(current < 10) {
        lcd.print("  ");
        lcd.print(float2s(current));
      } else {
        lcd.print(" ");
        lcd.print(float2s(current));
      }
    } else {
        lcd.print(float2s(current));
    }

    // power consumed
    lcd.setCursor(2, 2);
    if(ah < 10) {
      lcd.print(" ");
      lcd.print(float2s(ah));
    } else {
      lcd.print(float2s(ah));
    }

    // power regen
    lcd.setCursor(11, 2);
    if(ahRegen < 10) {
      lcd.print(" ");
      lcd.print(float2s(ahRegen));
    } else {
      lcd.print(float2s(ahRegen));
    }

    // temperature
    lcd.setCursor(2, 3);
    if(contTemp < 100) {
      if(contTemp < 10) {
        lcd.print("  ");
        lcd.print(float2s(contTemp));
      } else {
        lcd.print(" ");
        lcd.print(float2s(contTemp));
      }
    } else {
        lcd.print(float2s(contTemp));
    }

    // distance
    lcd.setCursor(10, 3);
    if(distance < 100) {
      if(distance < 10) {
        lcd.print("  ");
        lcd.print(float2s(distance));
      } else {
        lcd.print(" ");
        lcd.print(float2s(distance));
      }
    } else {
        lcd.print(float2s(distance));
    }
  }
     
}

void batterylevel(int xpos, int ypos, float consumed)
{
  float percent = consumed / (BATTERY_AH / 100);
  if(percent >= 0 && percent < 10)
  {
    byte batlevel[8] = {
    B01110,
    B11111,
    B10101,
    B10001,
    B11011,
    B11011,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 20 && percent >= 10)
  {
    byte batlevel[8] = {
    B01110,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 30 && percent > 20)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 45 && percent > 30)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B10001,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 55 && percent > 45)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B11111,
    B11111,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 70 && percent > 55)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent <= 85 && percent > 70)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
  if(percent > 85)
  {
    byte batlevel[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111,
    };
    lcd.createChar(0 , batlevel);
    lcd.setCursor(xpos,ypos);
    lcd.write(byte(0));
  }
}

char * float2s(float f) {
  char result[] = "";
  sprintf(result, "%.2f", f);
  return result;
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
