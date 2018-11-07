//************************************************************************************************************//
  
//********************* THE AUTO-ROTATE MATRIX WITH MPU6050 + KALMAN FILTER + BIT ANGLE MODULATION METHOD ********************//

//************************************************************************************************************//
#include <SPI.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "font8x8.h"
#include "bitmap.h"

#define blank_pin 3   // Defines actual BIT of PortD for blank - is Arduino UNO pin 3, MEGA pin 5
#define latch_pin 2   // Defines actual BIT of PortD for latch - is Arduino UNO pin 2, MEGA pin 4
#define clock_pin 13  // used by SPI, must be 13 SCK 13 on Arduino UNO, 52 on MEGA
#define data_pin 11   // used by SPI, must be pin MOSI 11 on Arduino UNO, 51 on MEGA

#define RowA_Pin 4
#define RowB_Pin 5
#define RowC_Pin 6
#define RowD_Pin 7

/////////////////////////////////////////////////////////////////////////////////////
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* MPU Data */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

byte red[4][16];
byte green[4][16];
byte blue[4][16];

int level=0;  // Keeps track of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

//************************************************************************************************************//

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473
#define dist(a, b, c, d) sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))
#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif
#define DEMO_RUNTIME  40000L
uint32_t timeStart;
//*********** Defining the Matrix *************

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G & B (4096 colours)
const  byte Size_X = 16;    // Number of Column X axis
const  byte Size_Y = 8;     // Number of Row Y axis

//************************************************************************************************************//

#define COLOUR_WHEEL_LENGTH 128

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;
byte myred, mygreen, myblue;

/**   An RGB color template */

struct Color
{
  unsigned char red, green, blue;

  Color(int r, int g, int b) : red(r), green(g), blue(b) {}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color tealcolor       = Color(0x00, 0x0F, 0x04);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color purplecolor     = Color(0x0F, 0x00, 0x0F);
const Color whitecolor      = Color(0x0F, 0x0F, 0x0F);
const Color blackcolor      = Color(0x00, 0x00, 0x00);

#define RED     0x0F,0x00,0x00
#define ORANGE  0x0F,0x04,0x00
#define YELLOW  0x0F,0x09,0x00
#define GREEN   0x00,0x0F,0x00
#define TEAL    0x00,0x0F,0x04
#define BLUE    0x00,0x00,0x0F
#define PURPLE  0x0F,0x00,0x0F
#define WHITE   0x0F,0x0F,0x0F
#define BLACK   0x00,0x00,0x00

void setup()
{

SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 10;

//pinMode (2, OUTPUT); // turn off PWM and set PortD bit 4 as output
//pinMode (3, OUTPUT); // turn off PWM and set PortD bit 5 as output
pinMode(latch_pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(RowA_Pin, OUTPUT);
pinMode(RowB_Pin, OUTPUT);
pinMode(RowC_Pin, OUTPUT);
pinMode(RowD_Pin, OUTPUT);

SPI.begin();
interrupts();

fill_colour_wheel();

//************************************************************************************************************//
  
//Serial.begin(115200);
Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  
//************************************************************************************************************//

}

void loop()
{
  clearfast(); 
  hScroll_MPU(0, orangecolor, bluecolor, "  Instructables       ");
  clearfast();
  hScroll_colorwheel_MPU(0, Color(0,0,0), "  Smart Matrix       ", 1, 35, 1);
}

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 15); 
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  //int WhichByte = Y;
  //int WhichBit = X;

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));

    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  }

}

void rowScan(byte row)
{
  
  if (row & 0x01) PORTD |= 1<<RowA_Pin;
    else PORTD &= ~(1<<RowA_Pin);
  
  if (row & 0x02) PORTD |= 1<<RowB_Pin;
    else PORTD &= ~(1<<RowB_Pin);

  if (row & 0x04) PORTD |= 1<<RowC_Pin;
    else PORTD &= ~(1<<RowC_Pin);

  if (row & 0x08) PORTD |= 1<<RowD_Pin;
    else PORTD &= ~(1<<RowD_Pin);
}

ISR(TIMER1_COMPA_vect){
  
PORTD |= ((1<<blank_pin));

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      
      //Blue
        
        myTransfer(blue[0][level]); 
      
      //Green
        
        myTransfer(green[0][level]);
      
      //Red
      
        myTransfer(red[0][level]);

      break;
    case 1:

      //Blue
        
        myTransfer(blue[1][level]);

      //Green

        myTransfer(green[1][level]);

      //Red

        myTransfer(red[1][level]);
        
      break;
    case 2:
      
      //Blue
        
        myTransfer(blue[2][level]);   
      
      //Green

        myTransfer(green[2][level]);   
      
      //Red

        myTransfer(red[2][level]);

      break;
    case 3:
      
      //Blue
        
        myTransfer(blue[3][level]); 
      
      //Green

        myTransfer(green[3][level]);
      
      //Red

        myTransfer(red[3][level]);
        
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(level);

PORTD |= 1<<latch_pin;
PORTD &= ~(1<<latch_pin);
delayMicroseconds(2); 
PORTD &= ~(1<<blank_pin);
//delayMicroseconds(5);

level++;

if(level==16)
level=0;
pinMode(blank_pin, OUTPUT);

}

inline static uint8_t myTransfer(uint8_t C_data){
  SPDR = C_data;
  asm volatile("nop"); 
  asm volatile("nop");
}


void clearfast ()
{
for (unsigned char j=0; j<16; j++)
        {
        red[0][j]   = 0;
        red[1][j]   = 0;
        red[2][j]   = 0;
        red[3][j]   = 0;
        green[0][j] = 0;
        green[1][j] = 0;
        green[2][j] = 0;
        green[3][j] = 0;
        blue[0][j] = 0;
        blue[1][j] = 0;
        blue[2][j] = 0;
        blue[3][j] = 0;
        }
}


//************************************************************************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
}

//************************************************************************************************************//

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}

//************************************************************************************************************//

void printChar(uint8_t x, uint8_t y, Color For_color, Color Bk_color, char ch)
{
  uint8_t xx,yy;
  xx=0;
  yy=0; 
  
  for (yy=0; yy < 8; yy++)
    {
    for (xx=0; xx < 8; xx++)
      {
      if (bitRead(pgm_read_byte(&font8x8[ch-32][7-yy]),7-xx)) // 4 == Font witdh -1
      
        {
        LED(x+xx,y+yy,For_color.red, For_color.green, For_color.blue);
        }
      else
        {
        LED(x+xx,y+yy, Bk_color.red, Bk_color.green, Bk_color.blue);        
        }
      }
    }
}


 unsigned int lenString(char *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}

byte getPixelChar(uint8_t x, uint8_t y, char ch)

{
    //ch = ch-32;
    if (x > 7) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font8x8[ch-32][7-y]),7-x); // 4 = Font witdh -1    
}

byte getPixelHString(uint16_t x, uint16_t y, char *p)
{
    p=p+x/7;
    return getPixelChar(x%7,y,*p);  
}

void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}

char flipbyte (char byte)
{
  char flop = 0x00;

  flop = (flop & 0b11111110) | (0b00000001 & (byte >> 7));
  flop = (flop & 0b11111101) | (0b00000010 & (byte >> 5));
  flop = (flop & 0b11111011) | (0b00000100 & (byte >> 3));
  flop = (flop & 0b11110111) | (0b00001000 & (byte >> 1));
  flop = (flop & 0b11101111) | (0b00010000 & (byte << 1));
  flop = (flop & 0b11011111) | (0b00100000 & (byte << 3));
  flop = (flop & 0b10111111) | (0b01000000 & (byte << 5));
  flop = (flop & 0b01111111) | (0b10000000 & (byte << 7));
  return flop;
}

//*******************************************************MPU*****************************************************//

void read_MPU6050()
{
 /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

//************************************************************************************************************//

void hScroll_colorwheel_MPU(uint8_t y, Color Bk_color, char *mystring, uint8_t font, uint8_t delaytime, uint8_t dir)
{
  int offset =0;

  Color setcolor, For_color;
   
for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1) ; (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
  {
    read_MPU6050();
      for (byte xx=0; xx<16; xx++)
      {
        for (byte yy=0; yy<8; yy++)
            {
              get_colour(colourPos + 8*(yy+xx), &For_color.red, &For_color.green, &For_color.blue);
              uint8_t pos = (kalAngleX >0 ? 1:0);
              if (pos)
              {
              if (getPixelHString(xx+offset,yy,mystring))
              
                setcolor = For_color;                
            
              else setcolor=Bk_color;

                LED(xx,(yy+y),setcolor.red, setcolor.green, setcolor.blue);
              }

              else
              {
                if (flipbyte(getPixelHString((xx+offset),yy, mystring)))
              
                setcolor = For_color;                
            
              else setcolor=Bk_color;

                LED(15-xx,7-(yy+y),setcolor.red, setcolor.green, setcolor.blue);
              }

            }
        }
        delay(delaytime); 
        increment_colour_pos(2); 
      }

}

void hScroll_MPU(uint8_t y, Color For_color, Color Bk_color, char *mystring)
{
    for (int offset=0; offset <((lenString(mystring)-8)*8-1); offset++)
      {
        read_MPU6050();
        uint8_t pos = (kalAngleX >0 ? 1:0);
        for (byte xx=0; xx<16; xx++)
          {
          for (byte yy=0; yy<8; yy++)
              {
                Color setcolor;
                uint8_t pos = (kalAngleX >0 ? 1:0);
                if (pos)
                  {
                    if (getPixelHString(xx+offset, yy, mystring)) 
                    setcolor = For_color; 
                    else setcolor=Bk_color;
                    LED(xx, yy+y, setcolor.red, setcolor.green, setcolor.blue);
                  }
                else
                  {
                    if (flipbyte(getPixelHString((xx+offset),yy, mystring)))             
                    setcolor = For_color;                           
                    else setcolor=Bk_color;
                    LED(15-xx,7-(yy+y),setcolor.red, setcolor.green, setcolor.blue);
                  }
              }
          }
          delay(40);  
      }
}
//************************************************************************************************************//
