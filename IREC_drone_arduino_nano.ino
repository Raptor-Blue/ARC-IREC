#include <Adafruit_Sensor.h>
#include <SimpleDHT.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MMA8451.h>
#include <Servo.h>
#include <Wire.h>


//create sensor objects and declare pins
/* DHT11
 *  VCC: 5V or 3V
 *  GND: GND
 *  DATA: 2
 */
int DHTpin = 2;
SimpleDHT11 dht11;
/* BMP180 - I2C connection
 *  3V: 3V 
 *  GND: GND
 *  SCL: pin A5
 *  SDA: pin A4
 */
Adafruit_BMP085 bmp;
/* MMA8451 - I2C connection
 *  Vin: 5V
 *  GND: GND
 *  SCL: pin A5
 *  SDA: pin A4
 */
 Adafruit_MMA8451 mma = Adafruit_MMA8451();
/* Servo
 *  Signal: pin 9 (or any pwm)
 */
Servo servo1;
servoPin = 9;

//function prototypes
void getBMPData();
void getDHTData();
void getMMAData();
void servoPosition();

void setup() {
  // put your setup code here, to run once:
  //information will be sent to an XBee module over serial
  Serial.begin(9600); 
  servo1.attach(servoPin);

  //setup bmp
  if (!bmp.begin()) {
  Serial.println("BMP initialization failed.");
  while (1) {}
  }
  Serial.println("BMP initialized.");
  
  //setup mma sensor
  if (! mma.begin()) {
    Serial.println("MMA8451 initialization failed.");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.println("MMA8451 initialized.");
  
}

void loop() {
  // put your main code here, to run repeatedly:

  getBMPData();
  getDHTData();
  getMMAData();
  
}

//functions
void getBMPData();
{
  //get pressure/altitude/temp
  Serial.println("BMP180 Readings");
  Serial.print("Temperature: ");
  Serial.print(bmp.readTemperature()); Serial.println(" C");
  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure()); Serial.println(" Pa");
  Serial.print("Altitude: ");
  Serial.print(bmp.readAltitude(101500)); Serial.println(" m");
  
  Serial.print("");
}

void getDHTData();
{
  //get temp/humidity
  byte temperature = 0;
  byte humidity = 0;

  if(dht11.read(DHTpin, &temperature, &humidity, NULL))
  {
    Serial.println("Read DHT11 failed.");
    return;
  }

  //display results
  Serial.println("DHT Readings: ");
  Serial.print("Temperature: "); Serial.print((int)temperature); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print((int)humidity); Serial.println(" %");
  
  Serial.println("");

  //DHT sampling rate is 1 Hz
  //delay(1000);
}

void getMMAData()
{
  //detect motion, tilt and basic orientation
  //Get a new sensor event  
  sensors_event_t event; 
  mma.getEvent(&event);

  //display results
  Serial.println("MMA sensor Readings: ");
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");

  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();

  switch (o)
  {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }
  Serial.println("");
}

void servoPosition();
{
  //write servo/servos to position to expand arms
}
