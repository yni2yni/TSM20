// ADS Touch Sensor Test Example Program (IC P/N:TSM20, 32QFN)
// Code:
// Date: 2023.07.11  Ver.: 0.0.2
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  2.1.1
// Author: Park, Byoungbae ( yni2yni@hanmail.net )
// Note: More information? Please ,send to e-mail.
// Uno R3, A4:SDA, A5: SCL, Leonardo 2:SDA,3:SCL, Uno R3, A4:SDA, A5: SCL,
// TSM20 VDD=3.3V (Nedd to Levle Converter, Arduino 5V <-> levle converter <-> TSM20 3.3V)
// Register setting values are subject to change without prior notice to improve touch operation.

#include <Wire.h>

#define LF     0x0A //New Line
#define CR     0x0D //Carriage  return
#define SPC    0x20 //Space


#define CH_en1    0x02 //Channel Control Register (0:Disable, 1: Enable)
#define CH_en2    0x03 //Channel Control Register (0:Disable, 1: Enable)
#define CH_en3    0x04 //Channel Control Register (0:Disable, 1: Enable)

#define Sensitivity1  0x11 //ch2,ch1  ex) 0xBB = Low: 0.95% , Middle: 1.20%, High: 1.60%
#define Sensitivity2  0x12 //ch4,ch3
#define Sensitivity3  0x13 //ch6,ch5
#define Sensitivity4  0x14 //ch8,ch7
#define Sensitivity5  0x15 //ch10,ch9
#define Sensitivity6  0x16 //ch12,ch11
#define Sensitivity7  0x17 //ch14,ch13
#define Sensitivity8  0x18 //ch16,ch15
#define Sensitivity9  0x19 //ch18,ch17
#define Sensitivity10  0x1A //ch20,ch19

#define CTRL1     0x22  //MS =0, FTC=01, ILC=00, RTC=010  
#define CTRL2     0x25  //Multi =0, IMP_SEL=0, FLT_SEL=0, MFM_SLE=01
#define CTRL3     0x28  //SRST =0 (Soft Reset Enable, Disable), SLEEP=0(Sleep Mode)

#define PWM_en1   0x05  //CS8 ~ CS1 PWM Mode Enable / Disable 
#define PWM_en2   0x06  //CS16 ~ CS9 PWM Mode Enable / Disable 
#define PWM_en3   0x07  //CS20 ~ CS17 PWM Mode Enable / Disable 
#define SINK_en   0x29  //SINK1_EN, SINK0_EN (COM0, COM1) 
#define SHD_en    0x2B  //SHD_EN
#define PWM_mduty 0x2C  //PWM Max Duty (max value=0b1011) 

#define PWM_duty1 0x2E  //CS2~CS1 LED PWM Duty
#define PWM_duty2 0x2F  //CS4~CS3 LED PWM Duty
#define PWM_duty3 0x30  //CS6~CS5 LED PWM Duty
#define PWM_duty4 0x31  //CS8~CS7 LED PWM Duty
#define PWM_duty5 0x32  //CS10~CS9 LED PWM Duty
#define PWM_duty6 0x33  //CS12~CS11 LED PWM Duty
#define PWM_duty7 0x34  //CS14~CS13 LED PWM Duty
#define PWM_duty8 0x35  //CS16~CS15 LED PWM Duty
#define PWM_duty9 0x36  //CS18~CS17 LED PWM Duty
#define PWM_duty10 0x37  //CS20~CS19 LED PWM Duty

#define RF_en  0x23  //RFID Detection Function  Enable/Disable
#define RF_sel 0x2A  //RFID Detection Channel, SINK_RF_SEL
#define EN_sel 0x3A  //EN Pin Function Enable/Disable

#define OUTPUT1   0x46 //cs4~cs1 output ex)00: No Output, 01: Low, 10: Middle, 11: High
#define OUTPUT2   0x47 //cs8~cs5 output
#define OUTPUT3   0x48 //cs12~cs9 output
#define OUTPUT4   0x49 //cs16~cs13 output
#define OUTPUT5   0x4A //cs20~cs17 output

#define Lock_mask1   0x3E //Lock Mask1 (Register write Lock bit)
#define Lock_mask2   0x3F //Lock Mask2 (Register write Lock bit)

// Hidden Register

// =============== TSM20 I2C Chip ID ================================================
#define TSM20_SLAVE_GND  0x68 //7bit address: 8bit address 0xD0>>1 //Chip_ID Pin = GND
#define TSM20_SLAVE_VDD  0x78 //7bit address: 8bit address 0xF0>>1 //Chip_ID Pin = VDD

void  Init_TSM20(void); //Initialize TSM20 (32QFN)

void  LED_ON(byte LED1, byte LED2, byte LED3); //PWM LED ON/OFF Control

byte Key_Data1 = 0x00; //touch Output Data , LED On/Off Data
byte Key_Data2 = 0x00;
byte Key_Data3 = 0x40; //Bit6: PWM Mode Enable/Disable,(1=PWM Control Mode Enable)

#define RESET_PIN 7 //Reset pin (High Pulse)
#define EN_PIN 6    //I2C Enable Pin (TSM20 I2C Block Enable Pin, active Low)


void  LED_ON(byte LED1, byte LED2, byte LED3)
{

//-------------- PWM Register Unlock ------------------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask1)); // sends register address
   Wire.write(byte(0xCD)); // Unlock Register used for Shared PWM mode
   Wire.endTransmission(); //
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask2)); // sends register address
   Wire.write(byte(0x0B)); // Unlock Register used for Shared PWM mode
   Wire.endTransmission(); //
//-------------- LED PWM Mode Enable/Disable-----------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en1)); // sends register address
   Wire.write(byte(LED1)); // LED On/Off Data (bit'1'=ON. bit '0'=OFF)
   Wire.endTransmission(); //

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en2)); // sends register address
   Wire.write(byte(LED2)); // LED On/Off Data (bit'1'=ON. bit '0'=OFF)
   Wire.endTransmission(); //

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en3)); // sends register address
   Wire.write(byte(LED3)); // LED On/Off Data (bit'1'=ON. bit '0'=OFF)
   Wire.endTransmission(); //
   
} //PWM LED ON/OFF Control

//----------------------- Register Dump ---------------------------
void Register_Dump()
{
   byte read_data[1] = {0};

   for (int i = 0; i < 256; i += 16)
   {
      for (int j = 0; j <= 15; j++)
      {
         Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
         Wire.write((i + j));                     // sends register address
         Wire.endTransmission();                  // stop transmitting
         Wire.requestFrom(TSM20_SLAVE_GND, 1);   // data read (2 byte)
         read_data[0] = Wire.read();              //
         print2hex(read_data, 1);                 //
      }
      Serial.write(LF);
      Serial.write(CR);
   }
   delay(500);
}
//----------------------- Register Read ---------------------------
void print2hex(byte *data, byte length) //Print Hex format
{
   Serial.print("0x");
   for (int i = 0; i < length; i++)
   {
      if (data[i] < 0x10)
      {
         Serial.print("0");
      }
      Serial.print(data[i], HEX);
      Serial.write(SPC);
   }
}


void setup(){

   delay(100);            //wait for 100[msec]
   Wire.begin();          // join i2c bus (Address optional for master)
   Wire.setClock(20000); // 200Khz
   Serial.begin(115200);  // start serial for output (Spped)
   // SDA, SCL = Hi-z (need to pull-up Resistor)

   pinMode(RESET_PIN, OUTPUT);
   //pinMode(EN_PIN, OUTPUT);

  // IC H/W reset signal control ,Active Falling edge Reset
   digitalWrite(RESET_PIN, LOW);  // Reset pin = low
   digitalWrite(RESET_PIN, HIGH); // Reset pin = High
   delay(2);                      //Min: wait for 2[msec]
   digitalWrite(RESET_PIN, LOW);  // Reset pin = low 

   // digitalWrite(EN_PIN, LOW);  // I2C_EN pin = low  (Active Low)

   delay(200);    //wait for 200[msec]
   Init_TSM20(); //Initialize TSM20
   delay(100);    //wait for 100[msec]
  
}

// ------------ Main()--------------------------------------------------------------
void loop() {

   byte read_data[5] = {0};
     
   // Touch Key read
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(OUTPUT1));            // sends register address (0x46h~0x4Ah)
   Wire.endTransmission();              // stop 
   Wire.requestFrom(TSM20_SLAVE_GND, 5); // Key data read (5 byte)
   read_data[0] = Wire.read();          //Key 4~1
   read_data[1] = Wire.read();          //Key 8~5
   read_data[2] = Wire.read();          //Key 12~9
   read_data[3] = Wire.read();          //Key 16~13
   read_data[4] = Wire.read();          //Key 20~17
  
   Serial.write(10);
   //Serial.print(" Touch Sensor Output Data (Hex) ---- > "); // Test Code
   print2hex(read_data, 5); // Test Code (Touch Output Data )
   Serial.write(LF);
   Serial.write(CR);

// 1 Key out data = 2bit('00', '01', '10', '11')
   if( (0x03 & read_data[0]) >= 0x02 ){ Key_Data1 |=0x01;}
   else{Key_Data1 &=0xFE;}
   if( (0x03 & (read_data[0] >>2)) >= 0x02){ Key_Data1 |=0x02;}
   else{Key_Data1 &=0xFD;}
   if( (0x03 & (read_data[0] >>4)) >= 0x02 ){ Key_Data1 |=0x04;}
   else{Key_Data1 &=0xFB;}
   if( (0x03 & (read_data[0] >>6)) >= 0x02 ){ Key_Data1 |=0x08;}
   else{Key_Data1 &=0xF7;}

   if( (0x03 & read_data[1]) >= 0x02 ){ Key_Data1 |=0x10;}
   else{Key_Data1 &=0xEF;}
   if( (0x03 & (read_data[1] >>2)) >= 0x02 ){ Key_Data1 |=0x20;}
   else{Key_Data1 &=0xDF;}
   if( (0x03 & (read_data[1] >>4)) >= 0x02 ){ Key_Data1 |=0x40;}
   else{Key_Data1 &=0xBF;}
   if( (0x03 & (read_data[1] >>6)) >= 0x02 ){ Key_Data1 |=0x80;}
   else{Key_Data1 &=0x7F;}

   if( (0x03 & read_data[2]) >= 0x02 ){ Key_Data2 |=0x01;}
   else{Key_Data2 &=0xFE;}
   if( (0x03 & (read_data[2] >>2)) >= 0x02){ Key_Data2 |=0x02;}
   else{Key_Data2 &=0xFD;}
   if( (0x03 & (read_data[2] >>4)) >= 0x02 ){ Key_Data2 |=0x04;}
   else{Key_Data2 &=0xFB;}
   if( (0x03 & (read_data[2] >>6)) >= 0x02 ){ Key_Data2 |=0x08;}
   else{Key_Data2 &=0xF7;}

   if( (0x03 & read_data[3]) >= 0x02 ){ Key_Data2 |=0x10;}
   else{Key_Data2 &=0xEF;}
   if( (0x03 & (read_data[3] >>2)) >= 0x02 ){ Key_Data2 |=0x20;}
   else{Key_Data2 &=0xDF;}
   if( (0x03 & (read_data[3] >>4)) >= 0x02 ){ Key_Data2 |=0x40;}
   else{Key_Data2 &=0xBF;}
   if( (0x03 & (read_data[3] >>6)) >= 0x02 ){ Key_Data2 |=0x80;}
   else{Key_Data2 &=0x7F;}

   if( (0x03 & read_data[4]) >= 0x02 ){ Key_Data3 |=0x01;}
   else{Key_Data3 &=0xFE;}
   if( (0x03 & (read_data[4] >>2)) >= 0x02){ Key_Data3 |=0x02;}
   else{Key_Data3 &=0xFD;}
   if( (0x03 & (read_data[4] >>4)) >= 0x02){ Key_Data3 |=0x04;}
   else{Key_Data3 &=0xFB;}
   if( (0x03 & (read_data[4] >>6)) >= 0x02){ Key_Data3 |=0x08;}
   else{Key_Data3 &=0xF7;}
      
   //LED On/Off Control
   LED_ON(Key_Data1, Key_Data2, Key_Data3 ); //LED On/Off Control
   Serial.print(" PWM LED ON/OFF  Data (hex) ---- > "); // Test Code
   // Serial.print(Key_Data1,HEX);
   // Serial.write(SPC);
   // Serial.print(Key_Data2,HEX);
   // Serial.write(SPC);
   // Serial.print(Key_Data3,HEX);
   Serial.write(LF);
   Serial.write(CR);

   delay(20);   
}

void  Init_TSM20(void)
{
 
//-------------- PWM Register Unlock ------------------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask1)); // sends register address
   Wire.write(byte(0xCD)); // Unlock Register used for Shared PWM mode
   Wire.endTransmission(); //
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask2)); // sends register address
   Wire.write(byte(0x0B)); // Unlock Register used for Shared PWM mode
   Wire.endTransmission(); //
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en1)); // sends register address
   Wire.write(byte(0xFF)); // PWM8~PWM1  Enable
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en2)); // sends register address
   Wire.write(byte(0xFF)); // PWM16~PWM9  Enable
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_en3)); // sends register address
   Wire.write(byte(0x4F)); // PWM20~PWM17  Enable
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(SINK_en)); // sends register address
   Wire.write(byte(0x38)); // SINK1, SINK0 Enable
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty1)); // sends register address
   Wire.write(byte(0xFF)); // PWM2, PWM1, Duty 
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty2)); // sends register address
   Wire.write(byte(0xFF)); // PWM4, PWM3, Duty 
   Wire.endTransmission(); // 
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty3)); // sends register address
   Wire.write(byte(0xFF)); // PWM6, PWM5, Duty 
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty4)); // sends register address
   Wire.write(byte(0xFF)); // PWM8, PWM7, Duty 
   Wire.endTransmission(); //

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty5)); // sends register address
   Wire.write(byte(0xFF)); // PWM10, PWM9, Duty 
   Wire.endTransmission(); //

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty6)); // sends register address
   Wire.write(byte(0xFF)); // PWM12, PWM11, Duty 
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty7)); // sends register address
   Wire.write(byte(0xFF)); // PWM14, PWM13, Duty 
   Wire.endTransmission(); // 
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty8)); // sends register address
   Wire.write(byte(0xFF)); // PWM16, PWM15, Duty 
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty9)); // sends register address
   Wire.write(byte(0xFF)); // PWM18, PWM17, Duty 
   Wire.endTransmission(); // 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(PWM_duty10)); // sends register address
   Wire.write(byte(0xFF)); // PWM20, PWM19, Duty 
   Wire.endTransmission(); // 

//-------------- Touch Register Unlock ------------------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask1)); // sends register address
   Wire.write(byte(0x6D)); // Unlock Register used for Touch Control Mode
   Wire.endTransmission(); //
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask2)); // sends register address
   Wire.write(byte(0x0A)); // Unlock Register used for Touch Control Mode
   Wire.endTransmission(); //

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL3)); // sends register address
   Wire.write(byte(0x02)); // S/W Enable , Sleep Mode = Disable
   //Wire.write(byte(0x03); // S/W Enable , Sleep Mode = Enable
   Wire.endTransmission(); // stop  

// ---------- Touch Sensing Channel ON/OFF ------------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CH_en1)); // sends register address
   Wire.write(byte(0xFF)); // CS8~ CS1 Enable (1: Touch Channel ON, 0: OFF)
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CH_en2)); // sends register address
   Wire.write(byte(0xFF)); // CS16~ CS9 Enable (1: Touch Channel ON, 0: OFF)
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CH_en3)); // sends register address
   Wire.write(byte(0x0F)); // CS20~ CS17 Enable (1: Touch Channel ON, 0: OFF)
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL1)); // sends register address
   Wire.write(byte(0x23)); // MS=0 Auto Mode, FTC=01, ILC=00,  RTC=010
   Wire.endTransmission(); // stop  

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x01)); // Multi Mode=0(Multi),IMP=0(High Imp.),
   Wire.endTransmission(); // FLT_SEL=0(Disable),MFM_SEL=01 Sense Frequency Select

// -------- Touch Sensitivity Setting (Touch Ouput threshold %)-----------
// Lower % values are more sensitive.
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity1));// sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity2)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 
  
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity3)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity4)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity5)); // sends register address
   Wire.write(byte(0xBB));// Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop    
  
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity6)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop    

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity7)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop    

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity8));// sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity9)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity10)); // sends register address
   Wire.write(byte(0xBB)); // Touch output threshold 0.85%, 1.2%, 1.6%
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(SHD_en)); // sends register address
   Wire.write(byte(0x10)); // Shared PWM Mode Enable
   Wire.endTransmission(); // stop    

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(EN_sel)); // sends register address
   Wire.write(byte(0x01)); // I2C_EN Port Function is Disable
   Wire.endTransmission(); // stop 

   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL3)); // sends register address
   Wire.write(byte(0x01)); // S/W Reset Disable , Sleep Mode = Enable
   //Wire.write(byte(0x00)); // S/W Reset Disable , Sleep Mode = Disable
   Wire.endTransmission(); // stop  

//--------------------- All Register Lock --------------------------------
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask1)); // sends register address
   Wire.write(byte(0x00)); // Register Write Lock enable
   Wire.endTransmission(); //
   
   Wire.beginTransmission(TSM20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask2)); // sends register address
   Wire.write(byte(0x00)); // Register Write Lock enable
   Wire.endTransmission(); //

   }
// End