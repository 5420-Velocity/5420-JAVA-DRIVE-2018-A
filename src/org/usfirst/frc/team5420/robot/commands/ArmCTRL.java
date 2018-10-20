package org.usfirst.frc.team5420.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import org.usfirst.frc.team5420.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class ArmCTRL extends Command {

	boolean isDone = false;
	static PWMSpeedController Motor = null; // Global to class
	static Encoder Encoder = null; // Global to class
	int targetArm;
	static DigitalInput lowerLimit = null; // Global to class
	static DigitalInput upperLimit = null; // Global to class
	double speed = 0.2;
	private boolean tooHigh = false;
	private Date EStopEncoderTime;
	private DigitalInput SELECTED = null;
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param encoderArm  The tool to use when Lifting to get to the target.
	 */
	public ArmCTRL (PWMSpeedController MOTORIN, Encoder encoderArm, DigitalInput lowerLimit){
		ArmCTRL.Motor = MOTORIN;
		ArmCTRL.Encoder = encoderArm;
		ArmCTRL.lowerLimit = lowerLimit;
	}
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 */
	public ArmCTRL (PWMSpeedController MOTORIN, Encoder EncoderIn, DigitalInput UpperLimit, DigitalInput LowerLimit){
		this(MOTORIN, EncoderIn, LowerLimit);
		ArmCTRL.upperLimit = UpperLimit;
	}
	
	/**
	 * Action INIT
	 * 
	 * @param Speed      Speed used to drive the motor
	 * @param targetArm  The Target Encoder Value, Should not be a Negative Number, its not Orders its POS.
	 */
	public ArmCTRL (double Speed, int targetArm){
		targetArm = Math.abs(targetArm); // Get ABS of the Number.
		if( ArmCTRL.Encoder.getDistance() > targetArm ){
        	// Number is above the Target
			this.tooHigh = true;
			System.out.println("Arm Power is getting Negative");
			this.speed = -( Math.abs(Speed) );
			SELECTED = Robot.lowerLimit; // Set the Instance Setting to be the lower Limit.
        }
        else if( ArmCTRL.Encoder.getDistance() < targetArm ){
        	// Number is below the Target
        	this.tooHigh = false;
        	System.out.println("Arm Power is getting Posative");
        	this.speed = ( Math.abs(Speed) );
        	SELECTED = Robot.upperLimit; // Set the Instance Setting to be the upper Limit.
        }
		
		this.targetArm = targetArm;
		ArmCTRL.Motor.stopMotor(); // Stop Motors
	}	
	
	@Override
	public void initialize(){
		// Do not reset the Encoder Since it is all relative to the Physical Robot.
		// Setup the Safety Time.
				Calendar calculateDate = GregorianCalendar.getInstance();
				calculateDate.add(GregorianCalendar.SECOND, (int) 2); // Time to Check the Encoder Distance is not Zero
				EStopEncoderTime = calculateDate.getTime();
	}
	
	@Override
	protected void execute() {
		// TODO: Add in the Lift Encoder Detection and the Light sensor Detection.
		// TODO: Implement the same Saftey feture as the DriveCTRL has line 83.
		// Do the Drive Operation.
		
		System.out.println(ArmCTRL.Encoder.getDistance() + " " + this.targetArm);
		
		if( this.tooHigh ){
			// To High, Needs to go Down.
			System.out.println("Going Down, Too High");
			if(ArmCTRL.Encoder.getDistance() > this.targetArm || SELECTED.get()){
				// If the Arm is Lower than the Target
				ArmCTRL.Motor.set(this.speed);
			}
			else {
				ArmCTRL.Motor.set(0);
				this.isDone = true;
			}
		}
		else {
			System.out.println("Going Up, Too Low");
			// To Low, Needs to go up.
			if(ArmCTRL.Encoder.getDistance() < this.targetArm || SELECTED.get()){
				// If the Arm is Lower than the Target
				ArmCTRL.Motor.set(this.speed);
			}
			else {
				ArmCTRL.Motor.setSpeed(0);
				this.isDone = true;
			}
		}
		
		
		// Safety Code, Its made to catch the Human Error of not plugging in the Encoder.
		//  Enocders will send a 0 value if you don't have an encoder plugged-in to the port.
		// Do the Safe Check to see if the Encoders are doing their thing or not after x seconds
		if( new Date().after(EStopEncoderTime) ) {
			// If the Encoder is not Past 10 ticks.
			if( Math.abs(ArmCTRL.Encoder.getDistance()) <= 10 ){
				System.err.println("WARN:    Encoder Detection Timeout [ArmCTRL]");
				ArmCTRL.Motor.setSpeed(0);
				this.isDone = true;
			}
		}
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}
