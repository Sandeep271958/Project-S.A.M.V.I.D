// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


float ax = 0, ay = 0, az = 0;
float wx = 0, wy = 0, wz = 0;
float qw = 0, qx = 0, qy = 0, qz = 0;
float gx = 0, gy = 0, gz = 0;


Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(1000);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(100); }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
 
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }

  

}


void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno08x.enableReport(SH2_ROTATION_VECTOR);

    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    

    if(sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
    }

    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Angular velocity data
        wx = sensorValue.un.gyroscope.x;
        wy = sensorValue.un.gyroscope.y;
        wz = sensorValue.un.gyroscope.z;
    }

    else if(sensorValue.sensorId == SH2_ROTATION_VECTOR){
      // Extract quaternion data
        qw = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        gx = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)); // Roll
        gy = asin(2.0 * (qw * qy - qz * qx)); // Pitch
        gz = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); // Yaw

    }

  }

  else{
    return;
  }
  
  // Print both acceleration and angular velocity in vector format
  Serial.print("<");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  // Serial.println(">");

  
  Serial.print(wx);
  Serial.print(",");
  Serial.print(wy);
  Serial.print(",");
  Serial.print(wz);
  Serial.print(",");
  
  
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println(">");


}