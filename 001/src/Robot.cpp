#include <cstdint>
#include "WPILib.h"
#include <CANTalon.h>
#include <Timer.h>
#include <iostream>

using namespace std;

class Robot: public IterativeRobot
{
    Joystick joystickMain; // Drive
    Joystick joystickSecondary; //Mechanisms

    CANTalon FL;
    CANTalon FR;
    CANTalon RL;
    CANTalon RR;

    CANTalon ballMotor;
    CANTalon ropeMotor;

    double rampRate = 8.0; // Higher = Faster Acceleration

    double speedRatio = 2; //Ratio used to control speed

    float mov, mainMagRAW, mainMag, potSwerveAngle, WrapMinusPiToPlusPi;
    bool movingForward;

    double magnitude;

    double rightTrigger = 0.0;
    double leftTrigger = 0.0;

    DoubleSolenoid *ballSole = new DoubleSolenoid(6 ,7);
    DoubleSolenoid *gearSole = new DoubleSolenoid(4 ,5);
    DoubleSolenoid *gearDoorSole = new DoubleSolenoid(1 ,0);


    Timer *timer = new Timer();


//    Compressor *c = new Compressor(0);



public:
    Robot() :
        joystickMain(0),
        joystickSecondary(1),

        FL(1), // R -1
        FR(2), // R -1

        RL(4), // L -1
        RR(3), // L -1
        ballMotor(5),
        ropeMotor(6)
    {

        FL.SetVoltageRampRate(rampRate);
        FR.SetVoltageRampRate(rampRate);
        RL.SetVoltageRampRate(rampRate);
        RR.SetVoltageRampRate(rampRate);

        ballMotor.SetVoltageRampRate(9.0);
        ropeMotor.SetVoltageRampRate(9.0);

        //Camera Setup
//        CameraServer::GetInstance()->SetQuality(50);
//        CameraServer::GetInstance()->StartAutomaticCapture("cam0");
    }

private:
    void RobotInit()
    {
        CameraServer::GetInstance()->StartAutomaticCapture();
        CameraServer::GetInstance()->SetSize(CameraServer::GetInstance()->kSize640x480);
        timer->Start();
           timer->Reset();

//        ballMotor.SetInverted(true);
//        timer->Get();
    }

    void AutonomousInit()
    {

    }

    void AutonomousPeriodic()
    {
    }

    void TeleopInit()
    {
    }


    void TeleopPeriodic()
    {
        joystickInputsToDriving();
        ballMechanism();
        ropeMechanism();
        gearMechanism();
//        getTriggers();
    }

    // Set the speed of the motors on the right side
    void setRight(double value)
    {
        FL.Set(value);
        FR.Set(value);
    }



    // Combining both left motors to set speed to value

    void setLeft(double value)
    {
        RR.Set(-value);
        RL.Set(-value);
    }

    void ballMechanism()
    {
        if(joystickSecondary.GetRawButton(5))
        {
            ballMotor.Set(1.0);
        }
        else if(joystickSecondary.GetRawButton(6))
        {
            ballMotor.Set(-1.0);
        }
        else
        {
            ballMotor.Set(0.0);
        }

        if(joystickSecondary.GetRawButton(4))
        {
            setBallMechanismUp(); // up is taking balls in
        }
        else
        {
            setBallMechanismDown(); // down is shooting balls out
        }
    }

    void setBallMechanismDown(){
        ballSole->Set(DoubleSolenoid::Value::kReverse);
    }

    void setBallMechanismUp()
    {
        ballSole->Set(DoubleSolenoid::Value::kForward);
    }

    void getTriggers()
    {
        rightTrigger = joystickSecondary.GetRawAxis(3);
        leftTrigger = joystickSecondary.GetRawAxis(2);
//        cout << rightTrigger << " " << leftTrigger << endl;
    }

    void ropeMechanism()
    {
        getTriggers();

        if (rightTrigger > 0.3)
        { //Used for climbing up
            ropeMotor.Set(rightTrigger); //Maybe opposite
        }
        else if (leftTrigger > 0.3)
        { //Used for climbing down
            ropeMotor.Set(-leftTrigger);
        }
        else
        {
            ropeMotor.Set(0.0);
        }
    }

    void pushGearOut()
    {
        gearSole->Set(DoubleSolenoid::Value::kForward);
    }

    void pullGearIn()
    {
        gearSole->Set(DoubleSolenoid::Value::kReverse);
    }

    void openGearDoor()
    {
        gearDoorSole->Set(DoubleSolenoid::Value::kForward);
    }

    void lockGearDoor()
    {
        gearDoorSole->Set(DoubleSolenoid::Value::kReverse);
    }


    void spendTime(float seconds)
    {
        timer->Reset();
        while (1)
        {
            if (timer->Get() >= seconds)
            {
                return;
            }
        }
    }

    void gearMechanism()
    {
        if (joystickSecondary.GetRawButton(1))
        {
            dontMove();
            openGearDoor();
            spendTime(0.7);
            pushGearOut();
            spendTime(0.7);
            pullGearIn();
            spendTime(0.5);
            lockGearDoor();
        }
    }
    // Set speed ratio using buttons
    void setRatio()
    {
        speedRatio = 2;
        if (joystickMain.GetRawButton(7))
        {
            speedRatio = 1;
        }
        else if (joystickMain.GetRawButton(9))
        {
            speedRatio = 4;
        }
    }

    void topRightAreaTalonSpeeds(int x) // x = [0, 40]
    {
        setLeft(-magnitude);
        setRight(-magnitude*(40-x)/40);
    }

    void topLeftAreaTalonSpeeds(int x) // x = [0, 40]
    {
        setLeft(-magnitude*(40-x)/40);
        setRight(-magnitude);
    }

    void bottomRightAreaTalonSpeeds(int x) // x = [0, 40]
    {
        setRight(magnitude*(40-x)/40);
        setLeft(magnitude);
    }

    void bottomLeftAreaTalonSpeeds(int x) // x = [0, 40]
    {
        setRight(magnitude);
        setLeft(magnitude*(40-x)/40);
    }

    void goingStraight()
    {
        setRight(-magnitude);
        setLeft(-magnitude);
    }

    void goingBack()
    {
        setRight(magnitude);
        setLeft(magnitude);
    }



    void goingRight() // Pivoting right
    {
        setLeft(-magnitude/2);
        setRight(magnitude/2);
    }

    void goingHalfRight()
    {
        setLeft(-magnitude);
        setRight(0);
    }

    void goingLeft() // Pivoting left
    {
        setLeft(magnitude/2);
        setRight(-magnitude/2);
    }

    void goingHalfLeft()
    {
        setLeft(0);
        setRight(-magnitude);
    }

    void dontMove() {
        setLeft(0.0);
        setRight(0.0);
    }

    void mainRotationDrive() // turning and moving man, get it!!!!!
    {

        double realMag = joystickMain.GetMagnitude();//Get real magnitude
        magnitude = realMag/speedRatio; // Get magnitude adjusted by speedRatio [0, 1]

        double rotation = joystickMain.GetDirectionDegrees(); //Get rotation values from joystick [-180, 180]

        if(joystickMain.GetRawButton(1)) {
            magnitude *= -1;
            rotation *= -1;
        }



        //limiting starting point on joystick
        if(realMag > 0.2)
        {
            // Check rotation values
            if (rotation >= -25 && rotation  <= 25)
            {
                goingStraight();
            }
            else if ((rotation <= -155 && rotation >= -180) || (rotation >= 155 && rotation <= 180))
            {
                goingBack();
            }
            else if (rotation >= 65 && rotation <= 115)
            {
                goingRight();
            }
            else if (rotation >= -115 && rotation <= -65)
            {
                goingLeft();
            }
            else if (rotation >= 25 && rotation <= 65)
            {
                topRightAreaTalonSpeeds(rotation - 25);
            }
            else if (rotation <= -25 && rotation >= -65)
            {
                topLeftAreaTalonSpeeds(-rotation - 25);
            }
            else if (rotation <= 155 && rotation >= 115)
            {
                bottomRightAreaTalonSpeeds(40 - (rotation - 115));
            }
            else if (rotation >= -155 && rotation <= -115)
            {
                bottomLeftAreaTalonSpeeds(40 -(-rotation - 115));
            }
            else
            {
                dontMove();
            }
        }
        else
        {
            dontMove();
        }
    }

    void joystickInputsToDriving()
    {
        setRatio();
        mainRotationDrive();
    }

    void TestPeriodic()
    {
    }
};

START_ROBOT_CLASS(Robot);
