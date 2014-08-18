#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



#define PPM_PIN 4
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PRINT_RAW_DATA 0

// ================================================================
// ===                        VARIABLES                         ===
// ================================================================

bool blinkState = false;

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float throttle,roll,pitch;

//output vars
unsigned long timer = 0;
int rollMin = 70;
int rollMax = -70;
int pitchMin = -50;
int pitchMax = 60;
int throttleMin = 500;
int throttleMax = 170;

int channel1 = 1000;
int channel2 = 1000;
int channel3 = 1000;



// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================


void pulseOut(int pin, int micro) {
  digitalWrite(pin,HIGH);
  delayMicroseconds(micro);
  digitalWrite(pin,LOW); 
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
     
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(PPM_PIN,OUTPUT);
 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;
        throttle = analogRead(A0);
        
        
        #if PRINT_RAW_DATA == 1
          Serial.print(roll); Serial.print("x");
          Serial.print(pitch); Serial.print("x");
          Serial.print(throttle); Serial.print("\n");
        #elif PRINT_RAW_DATA == 0
          Serial.print(channel2); Serial.print("x");
          Serial.print(channel3); Serial.print("x");
          Serial.print(channel1); Serial.print("\n");
        #endif
  
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        if (digitalRead(A1)) Serial.println("\t\t\t\tButton");
    }
    
    //every 20 milliseconds, send out the PPM data
    if (timer + 20 < millis()) {
      channel3 = map(ypr[1] * 180/M_PI,pitchMin,pitchMax,0,1000);
      channel2 = map(ypr[2] * 180/M_PI,rollMin,rollMax,0,1000);
      channel1 = map(analogRead(A0),throttleMin,throttleMax,0,1000);
      pulseOut(PPM_PIN, 100);
      delayMicroseconds(channel1+1000);
      pulseOut(PPM_PIN, 100);
      delayMicroseconds(channel2+1000);
      pulseOut(PPM_PIN, 100);
      delayMicroseconds(channel3+1000);
      pulseOut(PPM_PIN, 100);
    }
}


