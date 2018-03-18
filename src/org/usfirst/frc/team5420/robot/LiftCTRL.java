package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class LiftCTRL extends Command {

	boolean isDone = false;
	static PWMSpeedController Motor = null;
	static Encoder LiftEncoder = null;
	private static SolenoidMap breakMap = null;
	int targetArm;
	double speed = 0.2;
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 */
	public LiftCTRL (PWMSpeedController MOTORIN, Encoder EncoderIn){
		LiftCTRL.Motor = MOTORIN;
		LiftCTRL.LiftEncoder = EncoderIn;
	}
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 * @param breakMap   Solenoid Map of the Double Solenoids
	 */
	public LiftCTRL (PWMSpeedController MOTORIN, Encoder EncoderIn, SolenoidMap breakMap){
		this(MOTORIN, EncoderIn);
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
		if( LiftCTRL.LiftEncoder.getDistance() < targetArm ){
        	// Number is above the Target
			this.speed = -( Math.abs(Speed) ); // Go Down
        }
        else if( LiftCTRL.LiftEncoder.getDistance() > targetArm ){
        	// Number is below the Target
        	this.speed = Math.abs(Speed) ; // Go Up
        }
		
		this.targetArm = targetArm;
		LiftCTRL.Motor.stopMotor(); // Stop Motors
	}	
	
	@Override
	public void initialize(){
		
	}
	
	@Override
	public void start(){
		// Do not reset the Encoder Since it is all relative to the Physical Robot.
	}
	
	@Override
	protected void execute() {
		// TODO: Add in the Lift Encoder Detection and the Light sensor Detection.
		// TODO: Implement the same Saftey feture as the DriveCTRL has line 83.
		// Do the Drive Operation.
		if(LiftCTRL.LiftEncoder.getDistance() <= this.targetArm ){
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
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}