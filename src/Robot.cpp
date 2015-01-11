#include "WPILib.h"
#include "IMU.h"
#include "IMUAdvanced.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class Robot : public SampleRobot
{
	NetworkTable *table;
	//IMUAdvanced *imu;
	IMU *imu;
	SerialPort *serial_port;
	bool first_iteration;

    // Channels for the wheels
    const static int frontLeftChannel	= 1;		// PWM Port Number
    const static int rearLeftChannel	= 0;		// PWM Port Number
    const static int frontRightChannel	= 3;		// PWM Port Number
    const static int rearRightChannel	= 2;		// PWM Port Number
    const static int joystickChannel	= 0;		// First and only Joystick
    const static int gyroChannel		= 0;		// Gyro in Analog Input "0"

	RobotDrive robotDrive;	// robot drive system
	Joystick stick;			// only joystick
//	Gyro driveGyro;			// Gyro instance for RobotDrive

public:
	float Kp = 0.03;		// Proportional constant for the Gyro
	Robot() :
			robotDrive(frontLeftChannel, rearLeftChannel,
					   frontRightChannel, rearRightChannel),	// these must be initialized in the same order
			stick(joystickChannel)							// as they are declared above.
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the right side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor,   true);	// you may need to change or remove this to match your robot
	}

	void RobotInit() {

		table = NetworkTable::GetTable("datatable");
		serial_port = new SerialPort(57600,SerialPort::kOnboard);
        uint8_t update_rate_hz = 50;
        //imu = new IMUAdvanced(serial_port,update_rate_hz);
        imu = new IMU(serial_port,update_rate_hz);
        if ( imu ) {
        	LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;
	}

	/**
	 *
	 * Wait for 10 seconds
	 */
	void Autonomous(void)
	{
		Wait(2.0); 				//    for 10 seconds
	}


	void OperatorControl(void)
	{
		robotDrive.SetSafetyEnabled(false);

		while (IsOperatorControl())
		{
			if ( first_iteration ) {
	            bool is_calibrating = imu->IsCalibrating();
	            if ( !is_calibrating ) {
	                Wait( 0.3 );
	                imu->ZeroYaw();
	                first_iteration = false;
	            }
			}
			SmartDashboard::PutBoolean( "IMU_Connected", imu->IsConnected());
			SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());
			SmartDashboard::PutNumber("IMU_Pitch", imu->GetPitch());
			SmartDashboard::PutNumber("IMU_Roll", imu->GetRoll());
			SmartDashboard::PutNumber("IMU_CompassHeading", imu->GetCompassHeading());
			SmartDashboard::PutNumber("IMU_Update_Count", imu->GetUpdateCount());
			SmartDashboard::PutNumber("IMU_Byte_Count", imu->GetByteCount());

			//SmartDashboard::PutNumber("IMU_Accel_X", imu->GetWorldLinearAccelX());
			//SmartDashboard::PutNumber("IMU_Accel_Y", imu->GetWorldLinearAccelY());
			//SmartDashboard::PutBoolean("IMU_IsMoving", imu->IsMoving());
			//SmartDashboard::PutNumber("IMU_Temp_C", imu->GetTempC());
            SmartDashboard::PutBoolean("IMU_IsCalibrating", imu->IsCalibrating());

        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.

    		float angle = imu->GetYaw();
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetZ(), -angle *Kp);

			Wait(0.2);				// wait for a while
		}
	}

	/**
	 * Runs during test mode
	 */
	void Test() {

	}

};

START_ROBOT_CLASS(Robot);


