package org.usfirst.frc.team2586.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class JustDrive extends Command {

	private Timer time;
	private CANTalon frontRightDrive;
	private CANTalon frontLeftDrive;
	private CANTalon rearRightDrive;
	private CANTalon rearLeftDrive;

	public JustDrive(CANTalon frontLeftDrive, CANTalon rearLeftDrive, CANTalon frontRightDrive,  CANTalon rearRightDrive) {
		// name thy talons

		frontRightDrive = frontRightDrive;
		frontLeftDrive = frontLeftDrive;
		rearRightDrive = rearLeftDrive;
		rearLeftDrive = rearRightDrive;

	}

	// init the timer and reset.start the thing
	protected void initialize() {
		time = new Timer();
  
		time.reset();
		time.start();

	}

	// if not ten go all
	// if not ten then go not at all
	protected void execute() {

		if (time.get() < 3) {
			
			frontRightDrive.set(0.7);
			frontLeftDrive.set(0.7);
			rearRightDrive.set(0.7);
			rearLeftDrive.set(0.7);

		} else {
			frontRightDrive.set(0);
			frontLeftDrive.set(0);
			rearRightDrive.set(0);
			rearLeftDrive.set(0);

		}

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

