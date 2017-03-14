package org.usfirst.frc.team2586.robot;

import java.nio.channels.Selector;
import java.util.Arrays;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.vision.USBCamera;


import com.ctre.CANTalon;

public class Robot extends IterativeRobot {

	// make sure we know exactly what port the joystick will be in 'cause if
	// it's in the wrong one the robot can't find it

	private static final int OMNI_JOYSTICK_PORT = 0;
	private static final int XBAX_CONTROL_PORT = 1;

	private static final int SHOOT_SPEED_ENCODER_PWM = 4;
	private static final int SHOOT_EM_UP_TALON_PWM = 9;

	private static final int IN_TAKE_TALON_PWM = 8;

	private static final int SHOOTER_SMOOTHING = 1;

	private static final int CLIMB_CLIMB_TALON_PWM = 2;
	
	//usb camera code from here on is commented out
	//private static int cam1;
	//private static int cam2;

	private double PERCENT_SHOOT_SPEED = 0.1;

	private static final double DEFAULT_SHOOT_SPEED = 50;

	// increase for smoother but less sensitive velocity
	private static final int SMOOTHING = 30;
	private static final double SIGNAL_FLOOR = 0.005;

	private static final int FEED_TALON_PWM = 7;
	private static final int FEED_ENCODER_PWM = 3;

	private static final String USB_CAMERA_NAME = "";
	private static final String USB_CAMERA_NAME_0 = "";

	// CANTalon refrence numbers
	private static final int FL = 3;
	private static final int FR = 0;
	private static final int RL = 1;
	private static final int RR = 2;

	// change as needed
	// private final double gyroScaler = 1;

	private CANTalon frontLeftDrive;
	private CANTalon frontRightDrive;
	private CANTalon rearLeftDrive;
	private CANTalon rearRightDrive;

	private Talon shootEmUp;
	private Counter shootSpeed;
	private PIDController shootPID;

	// private Encoder shootOpEncode;

	private Talon gearPickUp;

	private Talon climbClimb;

	private Talon feedTalon;
	private Encoder feedCoder;
	private PIDController feedPID;

	private Joystick omniJoy;
	private XboxController xbax;

	private SendableChooser autoSelect;

	private PowerDistributionPanel PDP;

	//private CameraServer camera;
	//private CameraServer server;
	//private UsbCamera Cameron;
	//private UsbCamera Cameron2;
	// private Encoder frontLeftEncoder;
	// private Encoder frontRightEncoder;
	// private Encoder rearLeftEncoder;
	// private Encoder rearRightEncoder;

	private double xVel = 0;
	private double yVel = 0;

	private ADIS16448_IMU imu;

	private RobotDrive hateDrive;
	private boolean isCalPress = false;

	boolean climbToggle = false;
	boolean shootToggle = false;
	boolean previousButton = false;
	boolean currentButton = false;
	boolean previousButtonShoot = false;
	boolean currentButtonShoot = false;

	// filtering algorithm

	private double[] previousSamplesX = new double[SMOOTHING];
	private double[] previousSamplesY = new double[SMOOTHING];
	private double[] prevoiusSamplesAngle = new double[SMOOTHING];
	private double[] previousSamplesShooter = new double[SHOOTER_SMOOTHING];
	private double gravX = 0;
	private double gravY = 0;
	private int samplesSinceCal = 0;
	private long lastTime = 0;
	private double gravAngle = 0;
	
	

	public void robotInit() {

		// these numbers correspond with the ID numbers of the individual
		// CANTalons
		// leave blank for now
		frontLeftDrive = new CANTalon(FL);
		frontRightDrive = new CANTalon(FR);
		rearLeftDrive = new CANTalon(RL);
		rearRightDrive = new CANTalon(RR);

		frontRightDrive.setInverted(true);
		rearRightDrive.setInverted(true);

		/*
		CameraServer camera = CameraServer.getInstance();
		
		camera.startAutomaticCapture(cam1);
		server = CameraServer.getInstance();
		server.startAutomaticCapture(cam2);
		*/
		
		// push to turn on gearPickUp; release to stop

		gearPickUp = new Talon(IN_TAKE_TALON_PWM);

		// shootOpEncode = new Encoder(SHOOT_OPTICAL_ENCODER_PWM, 6);

		climbClimb = new Talon(CLIMB_CLIMB_TALON_PWM);
		feedTalon = new Talon(FEED_TALON_PWM);
		feedCoder = new Encoder(FEED_ENCODER_PWM, 5);
		feedPID = new PIDController(0.1, 0.001, 0.0, 0.1, feedCoder, feedTalon);

		// shooting talon and shooting encoder

		shootEmUp = new Talon(SHOOT_EM_UP_TALON_PWM);
		shootEmUp.setInverted(true);
		shootSpeed = new Counter(SHOOT_SPEED_ENCODER_PWM);
		shootSpeed.setSamplesToAverage(10);
		shootSpeed.setDistancePerPulse(1);
		shootSpeed.setPIDSourceType(PIDSourceType.kRate);
		shootPID = new PIDController(0.005d, 0.0005d, 0.04d, shootSpeed,
				shootEmUp);
		shootPID.setContinuous(false);
		shootPID.setPercentTolerance(PERCENT_SHOOT_SPEED * 100);

		omniJoy = new Joystick(OMNI_JOYSTICK_PORT);
		xbax = new XboxController(XBAX_CONTROL_PORT);

		imu = new ADIS16448_IMU();

		// frontLeftEncoder = new Encoder(FRONT_LEFT_ENCODER_PWM, 0);
		// frontRightEncoder = new Encoder(FRONT_RIGHT_ENCODER_PWM, 1);
		// rearRightEncoder = new Encoder(REAR_RIGHT_ENCODER_PWM, 2);
		// rearLeftEncoder = new Encoder(REAR_LEFT_ENCODER_PWM, 3);

		hateDrive = new RobotDrive(frontRightDrive, rearRightDrive,
				frontLeftDrive, rearLeftDrive);

		PDP = new PowerDistributionPanel();

		shootPID.startLiveWindowMode();

		autoSelect = new SendableChooser();
		autoSelect.addDefault("Just Drive", new JustDrive(frontRightDrive,
				frontLeftDrive, rearLeftDrive, rearRightDrive));
		autoSelect.addObject("Gear Drop", new GearDrop(frontRightDrive, frontLeftDrive, rearRightDrive, rearLeftDrive, gearPickUp));

		SmartDashboard.putData("Selecterr", autoSelect);
		SmartDashboard.putData("Just Drive", new JustDrive(frontRightDrive,
				frontLeftDrive, rearLeftDrive, rearRightDrive));
		SmartDashboard.putData("Gear Drop", new GearDrop(frontRightDrive, frontLeftDrive, rearRightDrive, rearLeftDrive, gearPickUp));
	}

	// filters out noise on the Gyro

	@Override
	public void robotPeriodic() {
		super.robotPeriodic();

		for (int i = 1; i < SMOOTHING; i++) {
			previousSamplesX[i - 1] = previousSamplesX[i];
			previousSamplesY[i - 1] = previousSamplesY[i];
			prevoiusSamplesAngle[i - 1] = prevoiusSamplesAngle[i];

		}
		for (int i = 1; i < SHOOTER_SMOOTHING; i++) {
			previousSamplesShooter[i - 1] = previousSamplesShooter[i];
		}

		previousSamplesX[SMOOTHING - 1] = imu.getAccelX();
		previousSamplesY[SMOOTHING - 1] = imu.getAccelY();
		prevoiusSamplesAngle[SMOOTHING - 1] = imu.getAngle();

		previousSamplesShooter[SHOOTER_SMOOTHING - 1] = shootSpeed.getRate();
		lastTime = System.currentTimeMillis();

		double smoothRateX = Arrays.stream(previousSamplesX).reduce(0,
				(x, y) -> x + y)
				/ SMOOTHING;
		double smoothRateY = Arrays.stream(previousSamplesY).reduce(0,
				(x, y) -> x + y)
				/ SMOOTHING;
		double smoothRateAngle = Arrays.stream(prevoiusSamplesAngle).reduce(0,
				(x, y) -> x + y)
				/ SMOOTHING;
		samplesSinceCal++;

		if (omniJoy.getRawButton(5)) {
			imu.reset();
			xVel = 0;
			yVel = 0;
		}
		if (omniJoy.getRawButton(10) && !isCalPress) {
			imu.calibrate();
			isCalPress = true;
			samplesSinceCal = 0;

			for (int i = 0; i < SMOOTHING; i++) {
				previousSamplesX[i] = 0;
				previousSamplesY[i] = 0;
				prevoiusSamplesAngle[i] = 0;
			}

		}
		if (!omniJoy.getRawButton(10)) {
			isCalPress = false;
		}

		if (samplesSinceCal == SMOOTHING) {
			gravX = smoothRateX;
			gravY = smoothRateY;
			gravAngle = smoothRateAngle;
		} else if (samplesSinceCal > SMOOTHING) {
			smoothRateX = imu.getAccelX();
			smoothRateY = imu.getAccelY();

			double angleDiff = Math.toRadians(-(smoothRateAngle - gravAngle));

			double gravityXcomp = gravX * Math.cos(angleDiff) - gravY
					* Math.sin(angleDiff);
			double gravityYcomp = gravX * Math.sin(angleDiff) + gravY
					* Math.cos(angleDiff);

			double xVec = smoothRateX - gravityXcomp;
			double yVec = smoothRateY - gravityYcomp;
			if (Math.abs(xVec) > SIGNAL_FLOOR)
				xVel += xVec;
			if (Math.abs(yVec) > SIGNAL_FLOOR)
				yVel += yVec;
		}
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
		shootPID.enable();
		shootPID.setInputRange(0, 5000);

		SmartDashboard.putDouble("Shooter Speed", DEFAULT_SHOOT_SPEED);
	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {
		double shooterSpeed = SmartDashboard.getDouble("Shooter Speed", 50);

		double percErr = shooterSpeed * PERCENT_SHOOT_SPEED;
		double lowerBound = shooterSpeed - percErr;
		double upperBound = shooterSpeed + percErr;

		// THIS IS THE CORRECT DRIVE CODE
		double x = -valueWithDeadzone(omniJoy.getRawAxis(1), .1);
		double y = valueWithDeadzone(omniJoy.getRawAxis(0), .1);
		double rotation = -valueWithDeadzone(omniJoy.getRawAxis(2), .2);

		// xbox override
		// double x = -valueWithDeadzone(blackHand.getRawAxis(1), .1);
		// double y = valueWithDeadzone(blackHand.getRawAxis(0), .1);
		// double rotation =
		// -valueWithDeadzone(blackHand.getTriggerAxis(Hand.kLeft), 0.1)
		// + valueWithDeadzone(blackHand.getTriggerAxis(Hand.kRight), 0.1);

		// FWD Vector = 1*rateY(speed Y is going) *sin THETA
		// THETA = if/else statement; expected angle straight ahead(90*)
		// if THETA is <90 then THETA = THETA + (90 - THETA)
		// if THETA is >90 then THETA = THETA - (90 - THETA)

		hateDrive.mecanumDrive_Cartesian(x, y, rotation, 0);

		// if (climbToggle == true) {
		// frontRightDrive.set(0);
		// rearRightDrive.set(0);
		// frontLeftDrive.set(0);
		// rearLeftDrive.set(0);/
		// }

		if (PDP.getCurrent(FR) < 1 || PDP.getCurrent(FL) < 1
				|| PDP.getCurrent(RL) < 1 || PDP.getCurrent(RR) < 1) {
			xbax.setRumble(RumbleType.kLeftRumble, 1.0);
		} else {
			xbax.setRumble(RumbleType.kLeftRumble, 0.0); 
		}

		if (xbax.getRawButton(1)) {
			gearPickUp.set(1);
		} else {
			gearPickUp.set(0);
		}

		// toggle set false
		// if toggle was set false and button is get then toggle is set to true
		// and motor is activated
		// if toggle was set true and button is get then toggle is set to false
		// and motor is deactivated

		currentButton = xbax.getRawButton(2);
		if (currentButton && !previousButton) {
			climbToggle = !climbToggle;
			previousButton = true;
		}
		if (currentButton == false) {
			previousButton = false;
		}

		if (climbToggle == true) {
			climbClimb.set(xbax.getRawAxis(1));
		}

		currentButtonShoot = xbax.getRawButton(5);
		if (currentButtonShoot && !previousButtonShoot) {
			shootToggle = !shootToggle;
			previousButtonShoot = true;
		}
		if (currentButtonShoot == false) {
			previousButtonShoot = false;
		}

		if (shootToggle == true) {
			shootPID.enable();
			shootPID.setSetpoint(shooterSpeed);

			if (shootSpeed.getRate() > lowerBound
					&& shootSpeed.getRate() < upperBound) {
				feedTalon.set(1);

			} else {
				feedTalon.set(0);
				shootPID.disable();
			}

		}

		{

			SmartDashboard.putData("Shoot Motor Tuning", shootPID);

			SmartDashboard.putBoolean("Climber Toggle", climbToggle);

			double smoothSpeed = Arrays.stream(previousSamplesShooter).reduce(
					0, (a, z) -> a + z)
					/ SHOOTER_SMOOTHING;
			SmartDashboard.putNumber("Shoot Speed smooth", smoothSpeed);
			SmartDashboard.putNumber("Shooter RPM", smoothSpeed * 60);
			SmartDashboard
					.putNumber("Current Shooter Percent", shootEmUp.get());
		}
		// SmartDashboard.putData("IMU", imu);
		// SmartDashboard.putNumber("Xvel", xVel);
		// SmartDashboard.putNumber("Yvel", yVel);
		// SmartDashboard.putNumber("IMU Angle", imu.getAngle());
		// SmartDashboard.putNumber("IMU Rate", imu.getRate());
		// SmartDashboard.putNumber("IMU AngleX", imu.getAngleX());
		// SmartDashboard.putNumber("IMU AngleY", imu.getAngleY());
		// SmartDashboard.putNumber("IMU AngleZ", imu.getAngleZ());
		// SmartDashboard.putNumber("IMU RateX", imu.getRateX());
		// SmartDashboard.putNumber("IMU RateY", imu.getRateY());
		// SmartDashboard.putNumber("IMU RateZ", imu.getRateZ());
		// SmartDashboard.putNumber("IMU AccelX", imu.getAccelX());
		// SmartDashboard.putNumber("IMU AccelY", imu.getAccelY());
		// SmartDashboard.putNumber("IMU AccelZ", imu.getAccelZ());
		// SmartDashboard.putNumber("IMU MagX", imu.getMagX());
		// SmartDashboard.putNumber("IMU MagY", imu.getMagY());
		// SmartDashboard.putNumber("IMU MagZ", imu.getMagZ());
		// SmartDashboard.putNumber("IMU Pitch", imu.getPitch());
		// SmartDashboard.putNumber("IMU Roll", imu.getRoll());
		// SmartDashboard.putNumber("IMU Yaw", imu.getYaw());

	}

	// deadzone is cumulative errors between the joystick and the robot and this
	// allows us to remove it

	// private void set(CANTalon frontRightDrive2, CANTalon rearRightDrive2,
	// CANTalon rearLeftDrive2, CANTalon frontLeftDrive2) {
	// TODO Auto-generated method stub

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
