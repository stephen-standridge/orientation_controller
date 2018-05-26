#include <Wire.h>
/* #include <ESP8266WiFi.h> for ESP8266 board */
/* #include <WiFi.h> for the ESP32 board */
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>


/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/************************* WiFi Access Point *********************************/

#define KANEOHE_WLAN_SSID       "kaneohe"
#define KANEOHE_WLAN_PASS       "oahu1964"

#define PI_WLAN_SSID       "Pi_SS_AP"
#define PI_WLAN_PASS       "brc!2017"

#define GRAY_AREA_WLAN_SSID       "Gray Area Incubator"
#define GRAY_AREA_WLAN_PASS       "grandstand"

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
//const IPAddress outIp(192, 168, 1, 237); 

const IPAddress outIp(192, 168, 42, 11); 
const unsigned int outPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)



/************ Control State ******************/
extern bool print_publish = false;
extern bool print_message = true;
const char EULER_ORIENTATION = 0;
const char VECTOR = 1;
const char QUATERNION = 2;
const char MAGNOMETER = 3;
const char ACCELEROMETER = 4;


extern char controlType = 4;

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();


  WiFi.begin(PI_WLAN_SSID, PI_WLAN_PASS);
  Serial.print("Connecting to ");
  Serial.println(PI_WLAN_SSID);
//  WiFi.begin(GRAY_AREA_WLAN_SSID, GRAY_AREA_WLAN_PASS);
//  Serial.print("Connecting to ");
//  Serial.println(GRAY_AREA_WLAN_SSID);  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
    
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {
    Serial.println("BNO055 detected"); Serial.println("");    
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);  
}

uint32_t x=0;

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("disconnected from wifi");  
    WiFi.begin(PI_WLAN_SSID, PI_WLAN_PASS);
    delay(500);
    return;
  }  
  /* Optional: Display calibration status */
//  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  // Now we can publish stuff!
  switch(controlType) {
    case VECTOR:
      break;
    case QUATERNION: 
      publishQuaternion();
      break;
    case MAGNOMETER:
      publishMagnometer();
      break;      
    case ACCELEROMETER:
      publishAccelerometer();
      break;      
    case EULER_ORIENTATION:
    default:
      publishEuler();
      break;    
  }
}

void controlcallback(char *data, uint16_t len) {
  if (print_message) {
    Serial.print("control callback: ");
    Serial.println(data);
  }
}

void publishMagnometer() {
  /* Get a new sensor event */
  imu::Quaternion quat = bno.getQuat();
  Adafruit_BNO055::adafruit_vector_type_t vectorType = Adafruit_BNO055::VECTOR_MAGNETOMETER;
  imu::Vector<3> mag = bno.getVector(vectorType);

  OSCMessage msg("/feeds/magnometer");
  msg.add(float(mag.x()));
  msg.add(float(mag.y()));
  msg.add(float(mag.z()));
  
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();    
}

void publishQuaternion() {
  /* Get a new sensor event */
  imu::Quaternion quat = bno.getQuat();

  OSCMessage msg("/feeds/quaternion");
  if (false) {
      Serial.print("x: ");
      Serial.print(float(quat.x()));
      Serial.print("\ty: ");
      Serial.print(float(quat.y()));
      Serial.print("\tz: ");
      Serial.print(float(quat.z()));
      Serial.print("\ts: ");
      Serial.print(float(quat.w()));  
      Serial.println("");  
  }
  msg.add(float(quat.x()));
  msg.add(float(quat.y()));
  msg.add(float(quat.z()));
  msg.add(float(quat.w()));    
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();  
}

void publishEuler() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  OSCMessage msg("/feeds/euler");
  msg.add(event.orientation.x);
  msg.add(event.orientation.y);
  msg.add(event.orientation.z);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void publishAccelerometer() {
  /* Get a new sensor event */
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  OSCMessage msg("/feeds/acceleration_quaternion");
  msg.add(float(acceleration.x()));
  msg.add(float(acceleration.y()));
  msg.add(float(acceleration.z()));
  if (false) {
      Serial.print("x: ");
      Serial.print(float(acceleration.x()));
      Serial.print("\ty: ");
      Serial.print(float(acceleration.y()));
      Serial.print("\tz: ");
      Serial.print(float(acceleration.z()));
      Serial.println("");  
  }  
 imu::Quaternion quat = bno.getQuat();  
  msg.add(float(quat.x()));
  msg.add(float(quat.y()));
  msg.add(float(quat.z()));
  msg.add(float(quat.w()));    

  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();  
}

