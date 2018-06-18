package org.usfirst.frc.team5420.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class ArmCTRLHigh extends Command {

	boolean isDone = false;
	static PWMSpeedController Motor = null; // Global to class
	static DigitalInput lowerLimit = null; // Global to class
	static DigitalInput upperLimit = null; // Global to class
	double speed = 0.2;
	private Date EStopEncoderTime;
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param encoderArm  The tool to use when Lifting to get to the target.
	 */
	public ArmCTRLHigh (PWMSpeedController MOTORIN, DigitalInput lowerLimit){
		ArmCTRL.Motor = MOTORIN;
		ArmCTRL.lowerLimit = lowerLimit;
	}
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 */
	public ArmCTRLHigh (PWMSpeedController MOTORIN, DigitalInput UpperLimit, DigitalInput LowerLimit){
		this(MOTORIN, LowerLimit);
		ArmCTRL.upperLimit = UpperLimit;
	}
	
	/**
	 * Action INIT
	 * 
	 * @param Speed      Speed used to drive the motor
	 * @param targetArm  The Target Encoder Value, Should not be a Negative Number, its not Orders its POS.
	 */
	public ArmCTRLHigh (double Speed){
		System.out.println("Arm Power is getting Negative");
		this.speed = ( Math.abs(Speed) );
		ArmCTRL.Motor.stopMotor(); // Stop Motors
	}	
	
	@Override
	public void initialize(){
		// Do not reset the Encoder Since it is all relative to the Physical Robot.
		// Setup the Safety Time.
				Calendar calculateDate = GregorianCalendar.getInstance();
				calculateDate.add(GregorianCalendar.SECOND, (int) 10); // Time to Check the Encoder Distance is not Zero
				EStopEncoderTime = calculateDate.getTime();
	}
	
	@Override
	protected void execute() {
		if(ArmCTRL.upperLimit.get()){
			ArmCTRL.Motor.set(this.speed);
		}
		else if(ArmCTRL.lowerLimit.get()){
			this.speed = -(this.speed);
			ArmCTRL.Motor.set(this.speed);
		}
		else {
			ArmCTRL.Motor.setSpeed(0);
			this.isDone = true;
		}

		// Safety Code, Its made to catch the Human Error of not plugging in the Encoder.
		//  Enocders will send a 0 value if you don't have an encoder plugged-in to the port.
		// Do the Safe Check to see if the Encoders are doing their thing or not after x seconds
		if( new Date().after(EStopEncoderTime) ) {
			System.err.println("WARN:    Timeout Detection [ArmCTRLHigh]");
			ArmCTRL.Motor.setSpeed(0);
			this.isDone = true;
		}
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}