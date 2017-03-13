package org.usfirst.frc.team2586.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.CANTalon;

//read the notepad for unresolved problems/messages; cross off anything that gets fixed

public class Robotfeb10initdrivetest extends IterativeRobot {

	// the numbers will be the number of the port on the driver station or the
	// slot in the PWM board
	// the numbers are random until robot is built

	// make sure we know exactly what port the joystick will be in 'cause if
	// it's in the wrong one the robot can't find it

	private static final int OMNI_JOYSTICK_PORT = 1;

	private static final int SHOOT_SPEED_ENCODER_PWM = 2;
	private static final int SHOOT_EM_UP_TALON_PWM = 3;

	private static final int FRONT_LEFT_ENCODER_PWM = 5;
	private static final int FRONT_RIGHT_ENCODER_PWM = 6;
	private static final int REAR_LEFT_ENCODER_PWM = 7;
	private static final int REAR_RIGHT_ENCODER_PWM = 8;

	private static final int IN_TAKE_TALON_PWM = 9;

	private static final int CLIMB_CLIMB_TALON_PWM = 13;

	private static final int SHOOT_SPEED_CONSTANT = 3000;
	private static final double PERCENT_SHOOT_SPEED = 0.1;
	private static final double SHOOT_ERROR = SHOOT_SPEED_CONSTANT
			* PERCENT_SHOOT_SPEED;
	private static final double UPPER_BOUNDRY = SHOOT_SPEED_CONSTANT
			+ SHOOT_ERROR;
	private static final double LOWER_BOUNDRY = SHOOT_SPEED_CONSTANT
			- SHOOT_ERROR;

	private static final int FEED_TALON_PWM = 12;
	private static final int FEED_ENCODER_PWM = 11;
	
	private static final int FL = 0;
	private static final int FR = 1;
	private static final int RL = 2;
	private static final int RR = 3;

	// change as needed
	private final double gyroScaler = 1;

	private CANTalon frontLeftDrive;
	private CANTalon frontRightDrive;
	private CANTalon rearLeftDrive;
	private CANTalon rearRightDrive;
	private Talon shootEmUp;
	private Encoder shootSpeed;
	private PIDController shootPID;

	// wimpy talon for intake motor
	private Talon inTake;

	private Talon climbClimb;

	private Talon feedTalon;
	private Encoder feedCoder;

	private Joystick omniJoy;
	private XboxController blackHand;

	private Encoder frontLeftEncoder;
	private Encoder frontRightEncoder;
	private Encoder rearLeftEncoder;
	private Encoder rearRightEncoder;

	private double xVel = 0;
	private double yVel = 0;

	private ADIS16448_IMU imu;

	private RobotDrive hateDrive;

	public void robotInit() {

		// these numbers correspond with the ID numbers of the individual
		// CANTalons
		// leave blank for now
		frontLeftDrive = new CANTalon(FL);
		frontRightDrive = new CANTalon(FR);
		rearLeftDrive = new CANTalon(RL);
		rearRightDrive = new CANTalon(RR);

		// push to turn on intake; release to stop
		// puts value to SmartDash for the driver
/*
		inTake = new Talon(IN_TAKE_TALON_PWM);

		//climbClimb = new Talon(CLIMB_CLIMB_TALON_PWM
		feedTalon = new Talon(FEED_TALON_PWM);
		feedCoder = new Encoder(FEED_ENCODER_PWM, 5);

		// shooting talon and shooting encoder

		shootEmUp = new Talon(SHOOT_EM_UP_TALON_PWM);
		shootSpeed = new Encoder(SHOOT_SPEED_ENCODER_PWM, 4);
		shootPID = new PIDController(0.1, 0.001, 0.0, shootSpeed, shootEmUp);

		omniJoy = new Joystick(OMNI_JOYSTICK_PORT);

		imu = new ADIS16448_IMU();
*/
		frontLeftEncoder = new Encoder(FRONT_LEFT_ENCODER_PWM, 0);
		frontRightEncoder = new Encoder(FRONT_RIGHT_ENCODER_PWM, 1);
		rearRightEncoder = new Encoder(REAR_RIGHT_ENCODER_PWM, 2);
		rearLeftEncoder = new Encoder(REAR_LEFT_ENCODER_PWM, 3);

		//hateDrive = new RobotDrive(frontRightDrive, rearRightDrive,
		//		frontLeftDrive, rearLeftDrive);
		
		blackHand = new XboxController(0);
		double leftStick = blackHand.getRawAxis(0);
		double rightStick = blackHand.getRawAxis(2);
		frontLeftDrive.set(leftStick);
		rearLeftDrive.set(leftStick);
		frontRightDrive.set(rightStick);
		rearRightDrive.set(rightStick);
	}

	// Please start commenting the code so we know what is going on ~Dom.
	// got it bro ~Hay

	@Override
	public void robotPeriodic() {
		super.robotPeriodic();

		xVel = xVel + imu.getRateX();
		yVel = yVel + imu.getRateY();
	}

	public Robotfeb10initdrivetest() {

	}

	@Override
	public void autonomousInit() {
		// TODO Auto-generated method stub
		super.autonomousInit();
	}

	@Override
	public void teleopInit() {
		// TODO Auto-generated method stub
		super.teleopInit();
	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {

		// values of the axies on the joystick
		// the .number is sentistivity which will probs change once the 'bot's
		// built
		// the 'omnidirectional' joystick can rotate a wee bit for precise
		// movements
		double x = -valueWithDeadzone(omniJoy.getRawAxis(0), .1);
		double y = -valueWithDeadzone(omniJoy.getRawAxis(1), .1);
		double rotation = -valueWithDeadzone(omniJoy.getRawAxis(2), .4);

		// FWD Vector = 1*rateY(speed Y is going) *sin THETA
		// THETA = if/else statement; expected angle straight ahead(90*)
		// if THETA is <90 then THETA = THETA + (90 - THETA)
		// if THETA is >90 then THETA = THETA - (90 - THETA)

		hateDrive.mecanumDrive_Cartesian(x, y, rotation, 0);

		if (omniJoy.getRawButton(4)) {
			inTake.set(1);
		} else {
			inTake.set(0);
		}

		if (omniJoy.getRawButton(5)) {
			inTake.set(-1);
		} else {
			inTake.set(0);
		}

		if (omniJoy.getRawButton(3)) {
			climbClimb.set(1);
		} else {
			climbClimb.set(0);
		}

		SmartDashboard.putData("intake value", inTake);

		if (omniJoy.getRawButton(2)) {
			imu.reset();
		}

		SmartDashboard.putData("Shoot Motor Speed", shootPID);

		//

		if (omniJoy.getRawButton(1)) {
			shootPID.setSetpoint(SHOOT_SPEED_CONSTANT);
			if (shootSpeed.getRate() > LOWER_BOUNDRY
					&& shootSpeed.getRate() < UPPER_BOUNDRY) {
				feedTalon.set(.5);
			} else {
				feedTalon.set(0);
			}
		} else {
			shootPID.setSetpoint(0);
		}

		// Jeff 'fixed' it
		SmartDashboard.putData("IMU", imu);
		SmartDashboard.putNumber("IMU Angle", imu.getAngle());
		SmartDashboard.putNumber("IMU Rate", imu.getRate());
		SmartDashboard.putNumber("IMU AngleX", imu.getAngleX());
		SmartDashboard.putNumber("IMU AngleY", imu.getAngleY());
		SmartDashboard.putNumber("IMU AngleZ", imu.getAngleZ());
		SmartDashboard.putNumber("IMU RateX", imu.getRateX());
		SmartDashboard.putNumber("IMU RateY", imu.getRateY());
		SmartDashboard.putNumber("IMU RateZ", imu.getRateZ());
		SmartDashboard.putNumber("IMU AccelX", imu.getAccelX());
		SmartDashboard.putNumber("IMU AccelY", imu.getAccelY());
		SmartDashboard.putNumber("IMU AccelZ", imu.getAccelZ());
		SmartDashboard.putNumber("IMU MagX", imu.getMagX());
		SmartDashboard.putNumber("IMU MagY", imu.getMagY());
		SmartDashboard.putNumber("IMU MagZ", imu.getMagZ());
		SmartDashboard.putNumber("IMU Pitch", imu.getPitch());
		SmartDashboard.putNumber("IMU Roll", imu.getRoll());
		SmartDashboard.putNumber("IMU Yaw", imu.getYaw());

	}

	// deadzone is cumulative errors between the joystick and the robot and this
	// allows us to remove it

	public double valueWithDeadzone(double in, double dead) {
		dead -= dead * (Math.abs(in) / 1);
		if (-dead < in && in < dead)
			return 0;
		if (in < 0) {
			return in + dead;
		} else {
			return in - dead;
		}

	}

	public void testPeriodic() {

	}

}
