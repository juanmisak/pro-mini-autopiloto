#include <PinChangeInt.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>

Servo brushless1;
Servo brushless2;
Servo brushless3;
Servo brushless4;

int velocidadb1 = 0;
int velocidadb2 = 0;
int velocidadb3 = 0;
int velocidadb4 = 0;
int maxdif = 10;

// diferenciales
int db1 = 0;
int db2 = 0;
int db3 = 0;
int db4 = 0;
 // pulso respuesta
int pr = 10;

// compensacion por rotor
int compensacion = 1;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;

int led = 13;
int ch1_pin=2;
int ch2_pin=3;
int ch3_pin=4;
int ch4_pin=7;
int ch5_pin=8;
int ch6_pin=9;
volatile uint16_t ch6_start=0, ch6_global_count=0;
volatile uint16_t ch5_start=0, ch5_global_count=0;
volatile uint16_t ch4_start=0, ch4_global_count=0;
volatile uint16_t ch3_start=0, ch3_global_count=0;
volatile uint16_t ch2_start=0, ch2_global_count=0;
volatile uint16_t ch1_start=0, ch1_global_count=0;  
 
volatile uint8_t flag=LOW;//global flag
 
volatile uint8_t ch6_global_flag=LOW;//global flag
volatile uint8_t ch5_global_flag=LOW;//global flag
volatile uint8_t ch4_global_flag=LOW;//global flag
volatile uint8_t ch3_global_flag=LOW;//global flag
volatile uint8_t ch2_global_flag=LOW;//global flag
volatile uint8_t ch1_global_flag=LOW;//global flag
 
// the setup routine runs once when you press reset:
void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  // initialize the digital pin as an output.
  Serial.begin(9600);
  // ISR for all channels
  PCintPort::attachInterrupt(ch6_pin,ch6_count,CHANGE);
  PCintPort::attachInterrupt(ch5_pin,ch5_count,CHANGE);
  PCintPort::attachInterrupt(ch4_pin,ch4_count,CHANGE);
  PCintPort::attachInterrupt(ch3_pin,ch3_count,CHANGE);
  PCintPort::attachInterrupt(ch2_pin,ch2_count,CHANGE);
  PCintPort::attachInterrupt(ch1_pin,ch1_count,CHANGE);
  
  brushless1.attach(6);
  brushless2.attach(5);
  brushless3.attach(10);
  brushless4.attach(11);

  /*
   * 
   * brushless1.attach(5);
  brushless2.attach(6);
  brushless3.attach(10);
  brushless4.attach(11);
  */

 // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  arm();
}
 
// the loop routine runs over and over again forever:
void loop() 
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  volatile static uint16_t ch6_static_count;//local count
  volatile static uint16_t ch5_static_count;//local count
  volatile static uint16_t ch4_static_count;//local count
  volatile static uint16_t ch3_static_count;//local count
  volatile static uint16_t ch2_static_count;//local count
  volatile static uint16_t ch1_static_count;//local count
   
  volatile static uint8_t updateflags;//lcoal flag
  volatile static uint8_t ch6_update_flag;//lcoal flag
  volatile static uint8_t ch5_update_flag;//lcoal flag
  volatile static uint8_t ch4_update_flag;//lcoal flag
  volatile static uint8_t ch3_update_flag;//lcoal flag
  volatile static uint8_t ch2_update_flag;//lcoal flag
  volatile static uint8_t ch1_update_flag;//lcoal flag

  #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax/500.0); Serial.print("\t");
        Serial.println(ay/500.0); Serial.print("\t");
        //Serial.println(az/500.0); Serial.print("\t");
        //Serial.print(gx); Serial.print("\t");
        //Serial.print(gy); Serial.print("\t");
        //Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        //Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        //Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        //Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

  // 55 - 156;
  // 1012 -2032
   velocidadb1 = map(ch1_static_count, 1012, 2032, 55, 150);
   velocidadb2 = map(ch1_static_count, 1012, 2032, 55, 150);
   velocidadb3 = map(ch1_static_count, 1012, 2032, 55, 150);
   velocidadb4 = map(ch1_static_count, 1012, 2032, 55, 150);

/*
        y ^
          |
          |_____> x

        2    3
           X    >
        4    1
*/
     //brushless1.write(70);
     //brushless2.write(velocidadb2);
     //brushless3.write(velocidadb3);
     //brushless4.write(velocidadb4);
     
      //eje x
     if(ay < 0){
        brushless2.write(velocidadb2+abs(ay/500.0));
        brushless3.write(velocidadb3+abs(ay/500.0));
        //brushless1.write(velocidadb1-abs(ay/500.0));
        //brushless4.write(velocidadb4-abs(ay/500.0));
     }
      else if(ay>0){
          brushless1.write(velocidadb1+abs(ay/500.0));
          brushless4.write(velocidadb4+abs(ay/500.0));
          //brushless2.write(velocidadb2-abs(ay/500.0));
          //brushless3.write(velocidadb3-abs(ay/500.0));
      }
      /*
        2    3
           X    >
        4    1
      */
      
      //eje y
      if(ax>0){
        brushless2.write(velocidadb2+abs(ax/500.0));
        brushless4.write(velocidadb4+abs(ax/500.0));
        //brushless1.write(velocidadb1-abs(ax/500.0));
        //brushless3.write(velocidadb3-abs(ax/500.0));
      }
      else if(ax<0){
          brushless1.write(velocidadb1+abs(ax/500.0));
          brushless3.write(velocidadb3+abs(ax/500.0));
          //brushless2.write(velocidadb2-abs(ax/500.0));
          //brushless4.write(velocidadb4-abs(ax/500.0));
      }
      
     delay(20);
   
  if(flag)
  {
  noInterrupts();
  updateflags=flag;
  ch6_update_flag=ch6_global_flag;
  ch5_update_flag=ch5_global_flag;
  ch4_update_flag=ch4_global_flag;
  ch3_update_flag=ch3_global_flag;
  ch2_update_flag=ch2_global_flag;
  ch1_update_flag=ch1_global_flag;
   
  if(ch6_update_flag)
  {
  ch6_static_count=ch6_global_count;
  }
  if(ch5_update_flag)
  {
  ch5_static_count=ch5_global_count;
  }
  if(ch4_update_flag)
  {
  ch4_static_count=ch4_global_count;
  }
  if(ch3_update_flag)
  {
  ch3_static_count=ch3_global_count;
  }
  if(ch2_update_flag)
  {
  ch2_static_count=ch2_global_count;
  }
  if(ch1_update_flag)
  {
  ch1_static_count=ch1_global_count;
  }
  
  // Serial.print("ch6: ");
  Serial.println(ch6_static_count);
  //brushless1(map(ch6_static_count,));
  // Serial.print("\t");
   
  // Serial.print("ch5: ");
  Serial.println(ch5_static_count);
  // Serial.print("\t");
   
  // Serial.print("ch4: ");
  Serial.println(ch4_static_count);
  // Serial.print("\t");
   
  // Serial.print("ch3: ");
  Serial.println(ch3_static_count);
  // Serial.print("\t");
   
  // Serial.print("ch2: ");
  Serial.println(ch2_static_count);
  // Serial.print("\t");
   
  // Serial.print("ch1: ");
  Serial.println(ch1_static_count);
  Serial.print("\n");
   
  ch6_global_count=0;
  ch6_global_flag=0;
  ch6_update_flag=0;
   
  ch5_global_count=0;
  ch5_global_flag=0;
  ch5_update_flag=0;
   
  ch4_global_count=0;
  ch4_global_flag=0;
  ch4_update_flag=0;
   
  ch3_global_count=0;
  ch3_global_flag=0;
  ch3_update_flag=0;
   
  ch2_global_count=0;
  ch2_global_flag=0;
  ch2_update_flag=0;
   
  ch1_global_count=0;
  ch1_global_flag=0;
  ch1_update_flag=0;
  flag=0;
  interrupts();
   
  //use all the ch*_static_count for computation here.
  }

}
void ch6_count()
{
  //Serial.println(millis());
  if(digitalRead(ch6_pin)==HIGH)
  {
    ch6_start= micros();
  }
  else
  {
    ch6_global_count=(uint16_t)(micros()-ch6_start);
    flag=HIGH;
    ch6_global_flag=HIGH;
  }
}
void ch5_count()
{
  //Serial.println(millis());
  if(digitalRead(ch5_pin)==HIGH)
  {
    ch5_start= micros();
  }
  else
  {
    ch5_global_count=(uint16_t)(micros()-ch5_start);
    flag=HIGH;
    ch5_global_flag=HIGH;
  }
}
void ch4_count()
{
  //Serial.println(millis());
  if(digitalRead(ch4_pin)==HIGH)
  {
    ch4_start= micros();
  }
  else
  {
    ch4_global_count=(uint16_t)(micros()-ch4_start);
    flag=HIGH;
    ch4_global_flag=HIGH;
  }
}
void ch3_count()
{
//Serial.println(millis());
  if(digitalRead(ch3_pin)==HIGH)
  {
    ch3_start= micros();
  }
  else
  {
    ch3_global_count=(uint16_t)(micros()-ch3_start);
    flag=HIGH;
    ch3_global_flag=HIGH;
  }
}
void ch2_count()
{
  //Serial.println(millis());
  if(digitalRead(ch2_pin)==HIGH)
  {
    ch2_start= micros();
  }
  else
  {
    ch2_global_count=(uint16_t)(micros()-ch2_start);
    flag=HIGH;
    ch2_global_flag=HIGH;
  }
}

void ch1_count()
{
  //Serial.println(millis());
  if(digitalRead(ch1_pin)==HIGH)
  {
    ch1_start= micros();
  }
  else
  {
    ch1_global_count=(uint16_t)(micros()-ch1_start);
    flag=HIGH;
    ch1_global_flag=HIGH;
  }
}

void arm(){
 brushless1.write(0);
 brushless2.write(0);
 brushless3.write(0);
 brushless4.write(0);
 delay(100);
 
 brushless1.write(20);
 brushless2.write(20);
 brushless3.write(20);
 brushless4.write(20);
 delay(100);
}
