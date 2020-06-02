#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <ctype.h>

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0  

#define ConnectCheck 'c'
#define ReadData 'r'

//Initialisation of variables
SoftwareSerial mySerial(5, 4); // RX, TX
MPU9250_DMP imu;

char state;
bool imuConnected = false;
char deviceId;

void setup() {
    Wire.begin();
    mySerial.begin(9600);
}

void loop() {
    // if data is available to read
    if( mySerial.available() ){     
      // read it and store it in 'state'
      state = mySerial.read();
    }
    switch(state){
      case ConnectCheck:
        connectCheckController();
        break;
      case ReadData:
        readDataController();
        break;
      default:
        changedeviceIdController();
        delay(100);
        break;
    }                   
}

void connectCheckController(){
  connectCheck();
  state = 'z';
  mySerial.print(deviceId); mySerial.println("+Success");
  }

void readDataController(){
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() > 0 ){ 
    // If the Fifo is updated and data is ready
    if ( imu.updateFifo() == INV_SUCCESS){
      readData(imuConnected);
    }
  } else {
    mySerial.println("fifo not available");
    }
}

void changedeviceIdController(){
//  If the value is a number update the ID
  if (isDigit(state)){
    deviceId = state;
    state = 'z';
  }
}

void connectCheck(){
  // Check to see whether the imu is connected
  if (imu.begin() == INV_SUCCESS){
    
    
    mySerial.print(deviceId); mySerial.println("+Connected");
    
    imuConnected = true;
    
    
    mySerial.print(deviceId); mySerial.println("+Setting");
    
    // Enable 3 main sensors
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Set gyro to 2000 dps
    imu.setGyroFSR(2000); 

    // Set accel to +/-2g
    imu.setAccelFSR(2); 

    // Set LPF corner frequency to 25Hz
    imu.setLPF(25); 

    // Set sample rate to 50Hz
    imu.setSampleRate(50); 

    // Set mag sample rate to 50Hz
    imu.setCompassSampleRate(50);    
  
    // Set the digital motion processor
    imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | 
      DMP_FEATURE_GYRO_CAL       | 
      DMP_FEATURE_SEND_CAL_GYRO  | 
      50);   
             
    imu.configureFifo(INV_XYZ_GYRO |INV_XYZ_ACCEL);               

  } else {
    mySerial.print(deviceId);mySerial.println("+Error");
  }
}

void readData(bool imuConnected){
  if (imuConnected){

    // Update the imu values
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    
    // Use the calc functions to convert  into the proper units
    float accelX = imu.calcAccel(imu.ax);
    float accelY = imu.calcAccel(imu.ay);
    float accelZ = imu.calcAccel(imu.az);
    float gyroX = imu.calcGyro(imu.gx);
    float gyroY = imu.calcGyro(imu.gy);
    float gyroZ = imu.calcGyro(imu.gz);
    float magX = imu.calcMag(imu.mx);
    float magY = imu.calcMag(imu.my);
    float magZ = imu.calcMag(imu.mz);
  
    // Print all the data to the serial port
    mySerial.print(deviceId);
    mySerial.print("+Accelerometer=" + String(accelX) + "," +
      String(accelY) + "," + String(accelZ));
    mySerial.print(" Gyroscope=" + String(gyroX) + "," +
      String(gyroY) + "," + String(gyroZ));
    mySerial.print(" Magnetometer=" + String(magX) + "," +
      String(magY) + "," + String(magZ));
    mySerial.println(" Time " + String(imu.time));
  }
}
