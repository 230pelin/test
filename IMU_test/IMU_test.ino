#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno; //= Adafruit_BNO055(55);
int resetPin = A6;

void reset() {
  Serial.println("Resetting.");
  digitalWrite(resetPin, LOW);

  delayMicroseconds(30);

  digitalWrite(resetPin, HIGH);

  bno.begin();
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, HIGH);

  /* Initialise the sensor */
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop(void)
{
//  if(millis() > 5000 && millis() < 5100){
//    reset();
//  }

  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  /* Display the floating point data */
  Serial.print("\tX: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.println(event.orientation.z, 4);

  imu::Quaternion quat = bno.getQuat();

  /* Display the quat data */
  //  Serial.print("\tqW: ");
  //  Serial.print(quat.w(), 4);
  //  Serial.print("\tqX: ");
  //  Serial.print(quat.y(), 4);
  //  Serial.print("\tqY: ");
  //  Serial.print(quat.x(), 4);
  //  Serial.print("\tqZ: ");
  //  Serial.print(quat.z(), 4);

  /* Display the current temperature */
  //  int8_t temp = bno.getTemp();

  //  Serial.print("\tCurrent Temperature: ");
  //  Serial.print(temp);
  //  Serial.println("\tC");
  //  Serial.println("");
}
