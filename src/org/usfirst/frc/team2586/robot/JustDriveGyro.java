package org.usfirst.frc.team2586.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class JustDriveGyro extends Command {

	private Timer time;
	private CANTalon frontRightDrive;
	private CANTalon frontLeftDrive;
	private CANTalon rearRightDrive;
	private CANTalon rearLeftDrive;
	private Gyro myGyro;
	private RobotDrive loveDrive;
	double theX;
	double theY;
	double theRot;
	double myGyroHeading = 0;

	public JustDriveGyro(RobotDrive drive, Gyro theGyro) {
		// name thy talons
		loveDrive = drive;
		myGyro = theGyro;

	}

	// init the timer and reset.start the thing
	protected void initialize() {
		time = new Timer();
  
		time.reset();
		time.start();
		myGyro.reset();

	}

	// if not ten go all
	// if not ten then go not at all
	protected void execute() {
		
		if (time.get() < 3) {
			theX = 0;
			theY = -1;
			theRot = 0;
			

		} else {
			theX = 0;
			theY = 0;
			theRot = 0;
		}
		
		double theError = (myGyroHeading) - (myGyro.getAngle());
		//double kP = SmartDashboard.getNumber("Gyro kP", .05);
		double theKP = .05;
		//SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
		if (theRot == 0) {
			myGyroHeading = myGyro.getAngle();

		} else {
			theRot = theRot + theKP * theError;
		}
		
		loveDrive.mecanumDrive_Cartesian(theX, theY, theRot, 0);

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}

