/* H3LIS331D_t3 Basic Example Code
 by: Kris Winer
 date: April 17, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic H3LIS331 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer data out. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 The H3LIS331DL is a low-power high-performance 3-axis linear accelerometer 
 belonging to the “nano” family, with digital I2C/SPI serial interface standard output. 
 The device features ultra-low power operational modes that allow advanced power saving and 
 smart sleep-to-wakeup functions.The H3LIS331DL has dynamically user-selectable full scales of ±100 g/±200g/±400g
 and it is capable of measuring accelerations with output data rates from 0.5 Hz to 1 kHz.
 The H3LIS331DL is available in a small thin plastic land grid array package (LGA) and it is 
 guaranteed to operate over an extended temperature range from -40 °C to +85 °C

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the H3LIS331D breakout board.
 
 Hardware setup:
 H3LIS331 Breakout ------ Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- pin 17 or 18
 SCL ----------------------- pin 16 or 19
 GND ---------------------- GND
 
  */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See also H3LIS331D data sheet: http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00053090.pdf
//
////////////////////////////
// H3LIS331D Registers //
////////////////////////////
#define  H3LIS331D_WHO_AM_I		0x0F
#define  H3LIS331D_CTRL_REG1		0x20
#define  H3LIS331D_CTRL_REG2		0x21
#define  H3LIS331D_CTRL_REG3		0x22
#define  H3LIS331D_CTRL_REG4		0x23
#define  H3LIS331D_CTRL_REG5		0x24
#define  H3LIS331D_HP_FILTER_RESET      0x25
#define  H3LIS331D_REFERENCE 		0x26
#define  H3LIS331D_STATUS_REG 		0x27
#define  H3LIS331D_OUT_X_L 		0x28
#define  H3LIS331D_OUT_X_H 		0x29
#define  H3LIS331D_OUT_Y_L 		0x2A
#define  H3LIS331D_OUT_Y_H 		0x2B
#define  H3LIS331D_OUT_Z_L 		0x2C
#define  H3LIS331D_OUT_Z_H 		0x2D
#define  H3LIS331D_INT1_CFG 		0x30
#define  H3LIS331D_INT1_SRC 		0x31
#define  H3LIS331D_INT1_THS 		0x32
#define  H3LIS331D_INT1_DURATION 	0x33
#define  H3LIS331D_INT2_CFG 		0x34
#define  H3LIS331D_INT2_SRC 		0x35
#define  H3LIS331D_INT2_THS 		0x36
#define  H3LIS331D_INT2_DURATION 	0x37

#define H3LIS331D_ADDRESS  0x18 // Address of H3LIS331D accelerometer

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
  AFS_100G = 0,
  AFS_200G,
  AFS_400G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_50Hz = 0,
  AODR_100Hz,
  AODR_400Hz,
  AODR_1000Hz
};

enum Pmode {  // set of allowable gyro sample rates
  PowerDown = 0,
  NormalMode,
  LowPower_0_5Hz,
  LowPower_1Hz,
  LowPower_2Hz,
  LowPower_5Hz,
  LowPower_10Hz
};

// Specify sensor full scale
uint8_t Ascale = AFS_100G;   // accel full scale
uint8_t Aodr = AODR_100Hz;   // accel data sample rate
uint8_t Pmode = NormalMode;  // accel power mode
float aRes;                  // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 15;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;

int16_t accelCount[3];          // Stores the 16-bit signed accelerometer
float accelBias[3] = {0, 0, 0}; // Bias corrections for accelerometer
uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az; // variables to hold latest sensor data values 


void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
/*  
  display.begin(); // Initialize the display
  display.setContrast(40); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("H3LIS331D");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
*/

  // Read the WHO_AM_I registers, this is a good test of communication
  Serial.println("H3LIS331D 9-axis motion sensor...");
  byte c = readByte(H3LIS331D_ADDRESS, H3LIS331D_WHO_AM_I);  // Read WHO_AM_I register for H3LIS331D  
  Serial.println("H3LIS331D Hi-g accel"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x32, HEX);
  /*
  display.setCursor(20,0); display.print("H3LIS331D");
  display.setCursor(0,10); display.print("I AM"); display.print(c, HEX);  
  display.setCursor(0,20); display.print("I Should Be"); display.print(0xD4, HEX); 
  display.setCursor(0,30); display.print("I AM"); display.print(d, HEX);  
  display.setCursor(0,40); display.print("I Should Be"); display.print(0x49, HEX); 
  display.display();
  delay(1000); 
  */

  if (c == 0x32) // WHO_AM_I should always be 0x32 for the accelerometer
  {  
   Serial.println("H3LIS331D is online...");
 
   initH3LIS331D(); 
   Serial.println("H3LIS331D initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   // get sensor resolutions, only need to do this once
   getAres();
   Serial.print("accel sensitivity is "); Serial.print(1./aRes); Serial.println(" LSB/g");
  
   accelcalH3LIS331D(accelBias); // Calibrate accelerometer, load biases in bias registers
   Serial.println("accel biases (g)"); Serial.println(accelBias[0]); Serial.println(accelBias[1]); Serial.println(accelBias[2]);
  
  /* display.clearDisplay();
     
  display.setCursor(0, 0); display.print("H3LIS331Dbias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(1000); 
 */ 
 
  }
  else
  {
    Serial.print("Could not connect to H3LIS331D: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
 // if (readByte(H3LIS331D_ADDRESS, H3LIS331D_STATUS_REG) & 0x08) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
//  } 
  

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print(ax, 1);  
    Serial.print(" ay = "); Serial.print(ay, 1); 
    Serial.print(" az = "); Serial.print(az, 1); Serial.println(" g");
    }               
    
/*   
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
 
    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
    display.display();
*/


    digitalWrite(myLed, !digitalRead(myLed));
    count = millis();  
    }
}

//===================================================================================================================
//====== Set of useful function to access acceleration and temperature data
//===================================================================================================================

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 100 Gs (000), 200 Gs (001),400 gs (010)  
    case AFS_100G:
          aRes = 100.0/32768.0;
          break;
    case AFS_200G:
          aRes = 200.0/32768.0;
          break;
    case AFS_400G:
          aRes = 400.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(H3LIS331D_ADDRESS, H3LIS331D_OUT_X_L, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) ; // Form signed 16-bit integer for each sample  
    destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) ;
    destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) ;
}
       

void initH3LIS331D()
{  
   // set power mode (bits 7:5), sample rate (bits 4:3), and enable all axes (bits 2:0)
   writeByte(H3LIS331D_ADDRESS, H3LIS331D_CTRL_REG1, Pmode << 5 | Aodr << 3 | 0x07);
   // set block data update (bit 7), full scale (bits 5:4)
   writeByte(H3LIS331D_ADDRESS, H3LIS331D_CTRL_REG4, 0x80 | Ascale << 4); // enable bloack data update
  }


// Function which accumulates accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer bias registers.
void accelcalH3LIS331D(float * dest2)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t samples = 256, ii;
 
  Serial.println("Calibrating accel...");
 
  // now get the accelerometer bias
   for(ii = 0; ii < samples ; ii++) {            
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(H3LIS331D_ADDRESS, H3LIS331D_OUT_X_L, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample  
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
    
    delay(12);  // wait twelve milliseconds for next data at 100 Hz sample rate
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;
}


// I2C read/write functions for the H3LIS331Dand AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(0x80 | subAddress);     // Put slave register address in Tx buffer, include 0x80 for H3LIS331D multiple byte read
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

