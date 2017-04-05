package org.usfirst.frc.team2586.robot;

import java.nio.channels.Selector;
import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.CameraServer;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.vision.USBCamera;


import com.ctre.CANTalon;

public class Robot extends IterativeRobot {

	// make sure we know exactly what port the joystick will be in 'cause if
	// it's in the wrong one the robot can't find it

	private static final int OMNI_JOYSTICK_PORT = 0;
	private static final int XBAX_CONTROL_PORT = 1;

	//private static final int SHOOT_SPEED_ENCODER_PWM = 6;
	private static final int SHOOT_EM_UP_TALON_PWM = 9;

	private static final int IN_TAKE_TALON_PWM = 0;

	private static final int GEAR_POP_OUT_SOLENOID_PORT = 0;
	private static final int GEAR_POP_IN_SOLENOID_PORT = 1;
	private static final int GEAR_TRAY_OUT_SOLENOID_PORT = 2;
	private static final int GEAR_TRAY_IN_SOLENOID_PORT = 3;
	
	private static final int ENCODER_FR_PORT_A = 2;
	private static final int ENCODER_FR_PORT_B = 3;
	private static final int ENCODER_FL_PORT_A = 8;
	private static final int ENCODER_FL_PORT_B = 9;
	private static final int ENCODER_RR_PORT_A = 0;
	private static final int ENCODER_RR_PORT_B = 1;
	private static final int ENCODER_RL_PORT_A = 6;
	private static final int ENCODER_RL_PORT_B = 7;

	private static final int SHOOTER_SMOOTHING = 1;

	private static final int CLIMB_CLIMB_TALON_PWM = 2;

	private static final int GEAR_UP_BUTTON = 4;
	private static final int GEAR_DOWN_BUTTON = 1;
	private static final int CLIMB_BUTTON = 2;
	private static final int CLIMB_BUTTON_SLOW = 1;
	private static final int SHOOT_BUTTON = 5;
	private static final int GYRO_ENABLE_BUTTON = 11;
	private static final int GYRO_DISABLE_BUTTON = 12;

	private static int cam1 = 0;

	private double PERCENT_SHOOT_SPEED = 0.1;

	private static final double DEFAULT_SHOOT_SPEED = 50;

	// increase for smoother but less sensitive velocity
	private static final int SMOOTHING = 30;
	private static final double SIGNAL_FLOOR = 0.005;

	private static final int FEED_TALON_PWM = 7;
	//private static final int FEED_ENCODER_PWM = 3;

	private static final int ULTRASONIC_PORT = 1;

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

	//private Talon gearPickUp;

	private Solenoid gearPopOut;
	private Solenoid gearPopIn;
	private Solenoid gearTrayOut;
	private Solenoid gearTrayIn;

	private Talon climbClimb;

	private Talon feedTalon;
	//private Encoder feedCoder;
	private PIDController feedPID;

	private Joystick omniJoy;
	private XboxController xbax;

	private SendableChooser autoSelect;

	private PowerDistributionPanel PDP;

	private Gyro gyro = new AnalogGyro(0);

	private AnalogInput sanick = new AnalogInput(ULTRASONIC_PORT);
	double volts;
	double distance;
	private static final double SCALING = 0.009765625;

	//private DigitalInput gearLimitSwitch;

	private CameraServer camera;

	private Encoder encoderFL;
	private Encoder encoderFR;
	private Encoder encoderRL;
	private Encoder encoderRR;

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

	private Timer time;

	boolean gyroMode = false;
	double gyroHeading = 0;

	Command autoCommand;

	public void robotInit() {

		// these numbers correspond with the ID numbers of the individual
		// CANTalons
		// leave blank for now
		frontLeftDrive = new CANTalon(FL);
		frontRightDrive = new CANTalon(FR);
		rearLeftDrive = new CANTalon(RL);
		rearRightDrive = new CANTalon(RR);

		frontLeftDrive.setInverted(true);
		frontRightDrive.setInverted(true);

		CameraServer camera = CameraServer.getInstance();

		camera.startAutomaticCapture();

		// push to turn on gearPickUp; release to stop

		//gearPickUp = new Talon(IN_TAKE_TALON_PWM);

		gearPopOut = new Solenoid(GEAR_POP_OUT_SOLENOID_PORT);
		gearPopIn = new Solenoid(GEAR_POP_IN_SOLENOID_PORT);
		gearTrayOut = new Solenoid(GEAR_TRAY_OUT_SOLENOID_PORT);
		gearTrayIn = new Solenoid(GEAR_TRAY_IN_SOLENOID_PORT);

		// shootOpEncode = new Encoder(SHOOT_OPTICAL_ENCODER_PWM, 6);

		climbClimb = new Talon(CLIMB_CLIMB_TALON_PWM);
		//feedTalon = new Talon(FEED_TALON_PWM);
		//feedCoder = new Encoder(FEED_ENCODER_PWM, 5);
		//feedPID = new PIDController(0.1, 0.001, 0.0, 0.1, feedCoder, feedTalon);

		// shooting talon and shooting encoder
		/*
		shootEmUp = new Talon(SHOOT_EM_UP_TALON_PWM);
		shootEmUp.setInverted(true);
		//shootSpeed = new Counter(SHOOT_SPEED_ENCODER_PWM);
		shootSpeed.setSamplesToAverage(10);
		shootSpeed.setDistancePerPulse(1);
		shootSpeed.setPIDSourceType(PIDSourceType.kRate);
		shootPID = new PIDController(0.005d, 0.0005d, 0.04d, shootSpeed,
				shootEmUp);
		shootPID.setContinuous(false);
		shootPID.setPercentTolerance(PERCENT_SHOOT_SPEED * 100);
		*/

		omniJoy = new Joystick(OMNI_JOYSTICK_PORT);
		xbax = new XboxController(XBAX_CONTROL_PORT);

		imu = new ADIS16448_IMU();

		encoderFL = new Encoder(ENCODER_FL_PORT_A, ENCODER_FL_PORT_B);
		encoderFR = new Encoder(ENCODER_FR_PORT_A, ENCODER_FR_PORT_B);
		encoderRR = new Encoder(ENCODER_RR_PORT_A, ENCODER_RR_PORT_B);
		encoderRL = new Encoder(ENCODER_RL_PORT_A, ENCODER_RL_PORT_B);
		
		encoderFL.setDistancePerPulse(152.0/22836.0);
		encoderFR.setDistancePerPulse(152.0/64247.0);
		encoderRL.setDistancePerPulse(-152.0/60352.0);
		encoderRR.setDistancePerPulse(1.0); //broken

		// hateDrive = new RobotDrive(frontRightDrive, rearRightDrive,
		// frontLeftDrive, rearLeftDrive);

		hateDrive = new RobotDrive(frontLeftDrive, rearLeftDrive,
				frontRightDrive, rearRightDrive);

		PDP = new PowerDistributionPanel();

		//shootPID.startLiveWindowMode();

		autoSelect = new SendableChooser();
		autoSelect.addDefault("Just Drive", new JustDrive(frontRightDrive,
				frontLeftDrive, rearLeftDrive, rearRightDrive));
		/*autoSelect.addObject("Gear Drop", new GearDrop(frontRightDrive,
				frontLeftDrive, rearRightDrive, rearLeftDrive,
				climbClimb, sanick));*/
		autoSelect.addObject("Just Drive Gyro", new JustDriveGyro(hateDrive,
				gyro));
		
		autoSelect.addObject("Center Gear", 
				new JustDriveGyroEncoder(hateDrive, gyro, encoderFR, encoderFL, encoderRR, encoderRL, gearPopOut, gearPopIn));
		
		
		autoSelect.addObject("SideGearLeft", 
				new SideGearLeft(hateDrive, gyro, encoderFR, encoderFL, encoderRR, encoderRL, gearPopOut, gearPopIn));
		autoSelect.addObject("SideGearRight", 
				new SideGearRight(hateDrive, gyro, encoderFR, encoderFL, encoderRR, encoderRL, gearPopOut, gearPopIn));
		
		SmartDashboard.putData("Selecterr", autoSelect);
		
		/*
		SmartDashboard.putData("Just Drive", new JustDrive(frontRightDrive,
				frontLeftDrive, rearLeftDrive, rearRightDrive));
		SmartDashboard.putData("Gear Drop", new GearDrop(frontRightDrive,
				frontLeftDrive, rearRightDrive, rearLeftDrive,
				climbClimb, sanick));
		SmartDashboard.putData("Just Drive Gyro", new JustDriveGyro(hateDrive,
				gyro));
		smartDashboard.putData("Just Drive Gyro Encoder",
				new JustDriveGyroEncoder(hateDrive, gyro, encoderFR, encod))
				*/
	}

	// filters out noise on the Gyro

	@Override
	public void robotPeriodic() {
		//super.robotPeriodic();
		/*
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
		*/
		
		
		/*
		 * SMART DASH DISPLAYS
		 */
		
		if (omniJoy.getRawButton(GYRO_ENABLE_BUTTON)) {
			gyroMode = true;
		} else if (omniJoy.getRawButton(GYRO_DISABLE_BUTTON)) {
			gyroMode = false;
		}
		
		SmartDashboard.putBoolean("Gyro mode", gyroMode);
		SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
		
		SmartDashboard.putNumber("encoderFR", encoderFR.getDistance());
		SmartDashboard.putNumber("encoderFL", encoderFL.getDistance());
		SmartDashboard.putNumber("encoderRR", encoderRR.getDistance());
		SmartDashboard.putNumber("encoderRL", encoderRL.getDistance());
		
		
		SmartDashboard.putNumber("Front left output", frontLeftDrive.get());
		SmartDashboard.putNumber("Front right output", frontRightDrive.get());
		SmartDashboard.putNumber("Rear left output", rearLeftDrive.get());
		SmartDashboard.putNumber("Rear right output", rearRightDrive.get());
		
		SmartDashboard.putNumber("Climb motor output", climbClimb.get());
		
		
	}

	@Override
	public void autonomousInit() {
		
		gyro.reset();
		
		// TODO Auto-generated method stub
		autoCommand = (Command) autoSelect.getSelected();
		autoCommand.start();
		// super.autonomousInit();
		/*
		 * time = new Timer();
		 * 
		 * time.reset(); time.start();
		 */
	}

	@Override
	public void teleopInit() {
		// TODO Auto-generated method stub
		//super.teleopInit();
		// shootPID.enable();
		//shootPID.setInputRange(0, 5000);

		//SmartDashboard.putNumber("Shooter Speed", DEFAULT_SHOOT_SPEED);
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		/*
		 * if (time.get() < 3) {
		 * 
		 * frontRightDrive.set(0.7); frontLeftDrive.set(0.7);
		 * rearRightDrive.set(0.7); rearLeftDrive.set(0.7);
		 * 
		 * } else { frontRightDrive.set(0); frontLeftDrive.set(0);
		 * rearRightDrive.set(0); rearLeftDrive.set(0);
		 * 
		 * }
		 */

	/*	if (time.get() < 3) {
			frontRightDrive.set(0.7);
			frontLeftDrive.set(0.7);
			rearRightDrive.set(0.7);
			rearLeftDrive.set(0.7);
		} else if (time.get() > 3) {
			frontRightDrive.set(0);
			frontLeftDrive.set(0);
			rearRightDrive.set(0);
			rearLeftDrive.set(0);
			gearPopOut.set(true);
		} else if (time.get() > 5) {
			frontRightDrive.set(-0.3);
			frontLeftDrive.set(-0.3);
			rearRightDrive.set(-0.3);
			rearLeftDrive.set(-0.3);
		} else {
			frontRightDrive.set(0);
			frontLeftDrive.set(0);
			rearRightDrive.set(0);
			rearLeftDrive.set(0);
	*/	}
	

	public void teleopPeriodic() {

		volts = sanick.getVoltage();
		distance = (volts / SCALING);

		SmartDashboard.putNumber("Sanick Volts", volts);
		SmartDashboard.putNumber("Sanick distance (in)", distance);
		/*
		double shooterSpeed = SmartDashboard.getDouble("Shooter Speed", 50);

		double percErr = shooterSpeed * PERCENT_SHOOT_SPEED;
		double lowerBound = shooterSpeed - percErr;
		double upperBound = shooterSpeed + percErr;
		*/

		// THIS IS THE CORRECT DRIVE CODE
		double x = valueWithDeadzone(omniJoy.getRawAxis(0), .1);
		double y = valueWithDeadzone(omniJoy.getRawAxis(1), .1);
		double rotation = valueWithDeadzone(omniJoy.getRawAxis(2), .2);

		/*
		 * Gyro is now in testing phase. Verify it works, then un-comment
		 * drive-assist, make sure there is a toggle, and try with drive assist.
		 * Next objective: gyro assist auton code.
		 */
		//SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
		//SmartDashboard.putBoolean("upper limit switch status",
			//	gearLimitSwitch.get());

		/*
		 * if (gearLimitSwitch.get()){ System.out.println("switch is on"); }
		 * else{ System.out.println("switch is off"); //hi }
		 */
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
		
		/*
		SmartDashboard.putBoolean("gyromode", gyroMode);
		if (omniJoy.getRawButton(GYRO_ENABLE_BUTTON)) {
			gyroMode = true;
		} else if (omniJoy.getRawButton(GYRO_DISABLE_BUTTON)) {
			gyroMode = false;
		}
		*/

		if (gyroMode) {
			double error = (gyroHeading) - (gyro.getAngle());
			double kP = SmartDashboard.getNumber("Gyro kP", .08);
			// SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
			if (Math.abs(rotation) > 0.05) {
				gyroHeading = gyro.getAngle();

			} else {
				rotation = rotation + kP * error;
			}
		}

		if (xbax.getRawButton(3) || omniJoy.getRawButton(9)) {
			gearPopOut.set(true);
			gearPopIn.set(false);
		} else {
			gearPopOut.set(false);
			gearPopIn.set(true);
		}

		
		
		SmartDashboard.putNumber("Rotation", rotation);

		hateDrive.mecanumDrive_Cartesian(x, y, rotation, 0);

		SmartDashboard.putNumber("X", x);
		SmartDashboard.putNumber("Y", y);
		SmartDashboard.putNumber("R", rotation);

		// PWM debug
		/*
		SmartDashboard.putNumber("Front left output", frontLeftDrive.get());
		SmartDashboard.putNumber("Front right output", frontRightDrive.get());
		SmartDashboard.putNumber("Rear left output", rearLeftDrive.get());
		SmartDashboard.putNumber("Rear right output", rearRightDrive.get());
		*/
		// if (climbToggle == true) {
		// frontRightDrive.set(0);
		// rearRightDrive.set(0);
		// frontLeftDrive.set(0);
		// rearLeftDrive.set(0);/
		// }

		/*
		 * if (PDP.getCurrent(FR) < 1 || PDP.getCurrent(FL) < 1 ||
		 * PDP.getCurrent(RL) < 1 || PDP.getCurrent(RR) < 1) {
		 * xbax.setRumble(RumbleType.kLeftRumble, 1.0); } else {
		 * xbax.setRumble(RumbleType.kLeftRumble, 0.0); }
		 */

		if (xbax.getRawButton(GEAR_UP_BUTTON) || omniJoy.getRawButton(7)) {
			gearTrayOut.set(true);
			gearTrayIn.set(false);
		} else {
			gearTrayIn.set(true);
			gearTrayOut.set(false);
		}
		
		if (xbax.getRawButton(CLIMB_BUTTON)|| omniJoy.getRawButton(10)){
			climbClimb.set(-1);

		}
		else if (xbax.getRawButton(CLIMB_BUTTON_SLOW)) {
			climbClimb.set(-0.5);
		}
		else{
			climbClimb.set(0);
		}
		
		
		/*
		 * while (gearLimitSwitch.get()){ gearPickUp.set(0); }
		 */

		// toggle set false
		// if toggle was set false and button is get then toggle is set to true
		// and motor is activated
		// if toggle was set true and button is get then toggle is set to false
		// and motor is deactivated
		
		/*
		currentButton = xbax.getRawButton(CLIMB_BUTTON)|| omniJoy.getRawButton(10);
		if (currentButton && !previousButton) {
			climbToggle = !climbToggle;
			previousButton = true;
		}
		if (currentButton == false) {
			previousButton = false;
		}

		if (climbToggle == true) {
			
			 if (xbax.getRawAxis(1)<0){ climbClimb.set(xbax.getRawAxis(1)); }
			 
			climbClimb.set(-1);
		} else if (xbax.getRawButton(3)) {
			climbClimb.set(-0.5);
		} else {
			climbClimb.set(0);
		}
		SmartDashboard.putBoolean("Climb toggle", climbToggle);
		*/
		
		/*
		 * currentButtonShoot = xbax.getRawButton(SHOOT_BUTTON); if
		 * (currentButtonShoot && !previousButtonShoot) { shootToggle =
		 * !shootToggle; previousButtonShoot = true; } if (currentButtonShoot ==
		 * false) { previousButtonShoot = false; }
		 * 
		 * if (shootToggle == true) { shootPID.enable();
		 * shootPID.setSetpoint(shooterSpeed);
		 * 
		 * if (shootSpeed.getRate() > lowerBound && shootSpeed.getRate() <
		 * upperBound) { feedTalon.set(1);
		 * 
		 * } else { feedTalon.set(0); shootPID.disable(); }
		 * 
		 * 
		 * 
		 * }
		 */

		/*
		 * SHOOTER CODE IS COMMENTED OUT {
		 * 
		 * SmartDashboard.putData("Shoot Motor Tuning", shootPID);
		 * 
		 * SmartDashboard.putBoolean("Climber Toggle", climbToggle);
		 * 
		 * double smoothSpeed = Arrays.stream(previousSamplesShooter).reduce( 0,
		 * (a, z) -> a + z) / SHOOTER_SMOOTHING;
		 * SmartDashboard.putNumber("Shoot Speed smooth", smoothSpeed);
		 * SmartDashboard.putNumber("Shooter RPM", smoothSpeed * 60);
		 * SmartDashboard .putNumber("Current Shooter Percent",
		 * shootEmUp.get()); }
		 */

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
		/*
		 * dead -= dead * (Math.abs(in) / 1); if (-dead < in && in < dead)
		 * return 0; if (in < 0) { return in + dead; } else { return in - dead;
		 * }
		 */
		return in;

	}

	public void testPeriodic() {

	}

}
