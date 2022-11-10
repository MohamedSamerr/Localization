#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu_readings" , &imu_msg);

geometry_msgs::Pose encoder_msg;
ros::Publisher pub_encoder("encoder_readings" , &encoder_msg);
//===============================================================
//===============================================================

#define outputA PA6
#define outputB PA7

double last_counter=0;
double counter=0;
double x_position = 0;
double y_position = 0;
float radius = 15;  //omniwheel radius                  
float omniwheel_angle  = 45;
float pulses_per_rotation = 540;

//===============================================================
//===============================================================

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL


// MPU control/status vars
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//TTL PINS
HardwareSerial Serial3(PB11,PB10);

void setup() 
{
    Serial3.begin(115200);
    (nh.getHardware())->setPort(&Serial3);
    (nh.getHardware())->setBaud(115200);

    nh.initNode();
    nh.advertise(pub_imu);
    nh.advertise(pub_encoder);
    //============================================================
    //============================================================

    pinMode(outputA,INPUT_PULLUP);
    pinMode(outputB,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(outputA), interrupt_A , CHANGE);
    attachInterrupt(digitalPinToInterrupt(outputB), interrupt_B , CHANGE);

    //============================================================
    //============================================================
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);


        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);


            imu_msg.orientation.w = q.w;
            imu_msg.orientation.x = q.x;
            imu_msg.orientation.y = q.y;
            imu_msg.orientation.z = q.z;       
        #endif


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


            imu_msg.angular_velocity.x = ypr[2] * 180/M_PI;   //roll
            imu_msg.angular_velocity.y = ypr[1] * 180/M_PI;   //pitch
            imu_msg.angular_velocity.z = ypr[0] * 180/M_PI;   //yaw
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


            imu_msg.linear_acceleration.x = aaReal.x;
            imu_msg.linear_acceleration.y = aaReal.y;
            imu_msg.linear_acceleration.z = aaReal.z;
        #endif
    }



    x_position = x_position + (((counter-last_counter)/pulses_per_rotation)*2*(22/7)*radius)
                              *(-sin(omniwheel_angle+imu_msg.angular_velocity.z));

    y_position = y_position + (((counter-last_counter)/pulses_per_rotation)*2*(22/7)*radius)
                              *(cos(omniwheel_angle+imu_msg.angular_velocity.z));

    encoder_msg.position.x = x_position; 

    encoder_msg.position.y = y_position;                        

    pub_imu.publish(&imu_msg);

    pub_encoder.publish(&encoder_msg);

    nh.spinOnce();

    delay(2);

}

// ================================================================
// ===                       INTERRUPTS                         ===
// ================================================================

void interrupt_A (void)
{
  if(digitalRead(outputA)!=digitalRead(outputB))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}
void interrupt_B (void)
{
  if(digitalRead(outputA)==digitalRead(outputB))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}
