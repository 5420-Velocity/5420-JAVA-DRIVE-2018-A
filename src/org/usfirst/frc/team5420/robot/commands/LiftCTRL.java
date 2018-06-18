package org.usfirst.frc.team5420.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import org.usfirst.frc.team5420.robot.EncoderMap;
import org.usfirst.frc.team5420.robot.SolenoidMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class LiftCTRL extends Command {

	boolean isDone = false;
	static PWMSpeedController Motor = null;
	static EncoderMap LiftEncoder = null;
	private static SolenoidMap breakMap = null;
	int targetArm;
	double speed = 0.2;
	private Date EStopEncoderTime;
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 */
	// TODO: Change Encoder to EncoderMap
	public LiftCTRL (PWMSpeedController MOTORIN, EncoderMap EncoderIn){
		LiftCTRL.Motor = MOTORIN;
		LiftCTRL.LiftEncoder = EncoderIn;
	}
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param encoderLiftMap  The tool to use when Lifting to get to the target.
	 * @param breakMap   Solenoid Map of the Double Solenoids
	 */
	public LiftCTRL (PWMSpeedController MOTORIN, EncoderMap encoderLiftMap, SolenoidMap breakMap){
		this(MOTORIN, encoderLiftMap);
		LiftCTRL.breakMap  = breakMap;
	}
	
	/**
	 * Action INIT
	 * 
	 * @param Speed      Speed used to drive the motor
	 * @param targetArm  The Target Encoder Value
	 */
	public LiftCTRL (double Speed, int targetArm){
		
    	// TODO: Check the Turn power is being check when you get the Robot
		// TODO: Do checking to see if it is past the point for the target and decide to move up or down.
		if( LiftCTRL.LiftEncoder.getDistance() > targetArm ){
        	// Number is above the Target
			System.out.println("Lift Power is getting Negative");
			this.speed = -( Math.abs(Speed) ); // Go Down
        }
        else if( LiftCTRL.LiftEncoder.getDistance() < targetArm ){
        	// Number is below the Target
        	System.out.println("Lift Power is getting Posative");
        	this.speed = Math.abs(Speed) ; // Go Up
        }
		
		this.targetArm = targetArm;
		LiftCTRL.Motor.stopMotor(); // Stop Motors
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
		if(LiftCTRL.LiftEncoder.getDistance() > this.targetArm ){
			if(LiftCTRL.breakMap != null){
				LiftCTRL.breakMap.close();
			}
			LiftCTRL.Motor.set(this.speed);
		}
		else {
			if(LiftCTRL.breakMap != null){
				LiftCTRL.breakMap.close();
			}
			LiftCTRL.Motor.set(0);
			this.isDone = true;
		}
		
		
		// Safety Code, Its made to catch the Human Error of not plugging in the Encoder.
		//  Enocders will send a 0 value if you don't have an encoder plugged-in to the port.
		// Do the Safe Check to see if the Encoders are doing their thing or not after x seconds
		if( new Date().after(EStopEncoderTime) ) {
			// If the Encoder is not Past 10 ticks.
			if( Math.abs(LiftCTRL.LiftEncoder.getDistance()) <= 10 ){
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