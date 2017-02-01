#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <TalonSRX.h>
#include <WPILib.h>
#include <Math.h>
#include <CameraServer.h>

#include <solenoid.h>
#include <Doublesolenoid.h>		//For whatever type of solonoid we use

class Robot: public frc::IterativeRobot {

public:
	void Disabled() {
		while(IsDisabled()) {}
	}

	void DisabledInit() {

	}
//	Default DisabledPeriodic() method... Overload me!
//	 Default RobotPeriodic() method... Overload me!
	void DisabledPeriodic() {}
	void RobotPeriodic() {}
	void TestInit() {}


	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		CameraServer::GetInstance()->StartAutomaticCapture();
		CameraServer::GetInstance()->SetSize(CameraServer::GetInstance()->kSize640x480);
	}


	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

	}


	void TeleopInit() {

	}

	void TeleopPeriodic() {
		double YAxis = joy->GetY(Joystick::kLeftHand);
		double XAxis = joy->GetX(Joystick::kLeftHand);

		double rot = std::atan(YAxis/XAxis);
		std::cout << rot << std::endl;
		talon1->Set(YAxis/2);

	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	frc::TalonSRX *talon1 = new frc::TalonSRX(1);
	frc::Joystick *joy = new frc::Joystick(0);
};

START_ROBOT_CLASS(Robot)
