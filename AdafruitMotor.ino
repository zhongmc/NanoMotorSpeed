#include <Wire.h>
#include <Adafruit_MotorShield.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield( 0x60 );  //0x40 pca9685 pwm 板的地址
Adafruit_DCMotor *leftMotor, *rightMotor;

void InitAdfMotor(uint8_t leftChannel, uint8_t rightChannel )
{

    leftMotor = AFMS.getMotor( leftChannel );
    rightMotor = AFMS.getMotor( rightChannel );
    AFMS.begin(); //AFMS.begin(50); //控制舵机时，需要50Hz的PWM信号；默认1.6Khz

}

void MoveAdfMotor(int leftSpeed, int rightSpeed )  //0-255 
{
    //SendMessages("MV adf moto:%d,%d\n", leftSpeed, rightSpeed);
    if( leftSpeed >= 0 )
    {
        leftMotor->run(FORWARD);
    }
    else
    {
        leftMotor->run(BACKWARD);
    }

    leftMotor->setSpeed(abs(leftSpeed));

    if( rightSpeed >= 0 )
    {
        rightMotor->run(FORWARD);
    }
    else
    {
        rightMotor->run(BACKWARD);
    }

    rightMotor->setSpeed(abs(rightSpeed));
    
}

void StopAdfMotor()
{
     leftMotor->run(RELEASE);
     rightMotor->run(RELEASE);
}
