package org.usfirst.frc.team2586.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;


public class SideGearLeft extends Command {

	private Timer time;
	private CANTalon frontRightDrive;
	private CANTalon frontLeftDrive;
	private CANTalon rearRightDrive;
	private CANTalon rearLeftDrive;
	
	private Encoder encFR;
	private Encoder encFL;
	private Encoder encRR;
	private Encoder encRL;
	
	private Solenoid popOut;
	private Solenoid popIn;
	
	private Gyro myGyro;
	private RobotDrive loveDrive;
	double theX;
	double theY;
	double theRot;
	double myGyroHeading = 0;
	
	int state = 0;

	public SideGearLeft(RobotDrive drive, Gyro theGyro, Encoder encoderFR, Encoder encoderFL, Encoder encoderRR, Encoder encoderRL, Solenoid gearPopOut, Solenoid gearPopIn) {
		// name thy talons
		loveDrive = drive;
		myGyro = theGyro;
		
		encFR = encoderFR;
		encFL = encoderFL;
		encRR = encoderRR;
		encRL = encoderRL;
		
		popOut = gearPopOut;
		popIn = gearPopIn;

	}

	// init the timer and reset.start the thing
	protected void initialize() {
		time = new Timer();
  
		time.reset();
		time.start();
		myGyro.reset();
		
		encFR.reset();
		encFL.reset();
		encRR.reset();
		encRL.reset();
		
		popIn.set(true);
		popOut.set(false);

	}

	// if not ten go all
	// if not ten then go not at all
	protected void execute() {
		switch (state){
		case 0:
			myGyro.reset(); //reset eveything
			
			encFR.reset();
			encFL.reset();
			encRR.reset();
			encRL.reset();
			
			time.reset();
			
			popIn.set(true);
			popOut.set(false);
			
			state++;
			break;
			
		case 1:
			if (encRL.getDistance() < 114.0-39.0) {//Move forward
				theX = 0;
				theY = -1;
				theRot = 0;
		

			} else {
				theX = 0;
				theY = 0;
				theRot = 0;
				time.reset();
				state++;
			}
			
			break;
		case 2:
			if (time.get()>0.5){//delay
				time.reset();
				state++;
			}
			break;
		case 3:
			if (myGyro.getAngle() < 50) {//turn 50 degrees
				theX = 0;
				theY = 0;
				theRot = 0;
				
				myGyroHeading = 50;
		

			} else {
				theX = 0;
				theY = 0;
				theRot = 0;
				encRL.reset();
				time.reset();
				state++;
			}
			break;
		case 4:
			if (time.get()>0.5){//Delay
				time.reset();
				state++;
			}
			break;
		case 5:
			if (encRL.getDistance() < 28.0){// move forward to place gear
				theX = 0;
				theY = -0.7;
				theRot = 0;
			}
			else {
				theX = 0;
				theY = 0;
				theRot = 0;
				state++;
			}
			if (time.get() > 2.0){
				theX = 0;
				theY = 0;
				theRot = 0;
				state++;
			}
			break;
		case 6:
			time.reset();
			state++;
		case 7:
			
			popOut.set(true); //place gear on peg
			popIn.set(false);
			if (time.get() > 0.25){
				popOut.set(false);
				popIn.set(true);
			}
			if (time.get() > 0.5){
				popOut.set(true);
				popIn.set(false);
			}
			
			if (time.get() > 1.0){
				myGyro.reset();
				
				encFR.reset();
				encFL.reset();
				encRR.reset();
				encRL.reset();
				
				state++;
			}
			break;
		/*
		case 8:
			if (encRL.getDistance() > -12.0) { //back up from peg
				theX = 0;
				theY = 1;
				theRot = 0;
		

			} else {
				theX = 0;
				theY = 0;
				theRot = 0;
				time.reset();
				state++;
			}
			break;
		case 9:
			if (encRL.getDistance() < 10.0) { //ram peg
				theX = 0;
				theY = -1;
				theRot = 0;
		

			} else {
				theX = 0;
				theY = 0;
				theRot = 0;
				time.reset();
				state++;
			}
			if (time.get() > 1.0){
				theX = 0;
				theY = 0;
				theRot = 0;
				time.reset();
				state++;
			}
			break;
		*/
		case 8:
			if (encRL.getDistance() > -12.0) { //back up from peg
				theX = 0;
				theY = 1;
				theRot = 0;
		

			} else {
				theX = 0;
				theY = 0;
				theRot = 0;
				time.reset();
				state++;
			}
			break;
		case 9:
			popIn.set(true);//retract pistons
			popOut.set(false);
			state++;
			break;
		case 10:
			
			theX = 0;
			theY = 0;
			theRot = 0;
			myGyro.reset();
			
			encFR.reset();
			encFL.reset();
			encRR.reset();
			encRL.reset();
			myGyroHeading = 0;
			
			break;
		}
		
		
		double theError = (myGyroHeading) - (myGyro.getAngle());
		//double kP = SmartDashboard.getNumber("Gyro kP", .05);
		double theKP = .05;
		//SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
		if (theRot != 0) {
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

