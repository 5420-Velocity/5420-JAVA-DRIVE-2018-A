package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class ArmCTRL extends Command {

	boolean isDone = false;
	static PWMSpeedController Motor = null; // Global to class
	static Encoder LiftEncoder = null; // Global to class
	int targetArm;
	static DigitalInput lowerLimit = null; // Global to class
	static DigitalInput upperLimit = null; // Global to class
	double speed = 0.2;
	private boolean tooHigh = false;
	
	/**
	 * Class Definition INIT
	 * 
	 * @param MOTORIN    The motor to Contorl
	 * @param EncoderIn  The tool to use when Lifting to get to the target.
	 */
	public ArmCTRL (PWMSpeedController MOTORIN, Encoder EncoderIn, DigitalInput lowerLimit){
		ArmCTRL.Motor = MOTORIN;
		ArmCTRL.LiftEncoder = EncoderIn;
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
		if( ArmCTRL.LiftEncoder.getDistance() < targetArm ){
        	// Number is above the Target
			this.speed = -( Math.abs(Speed) );
        }
        else if( ArmCTRL.LiftEncoder.getDistance() > targetArm ){
        	// Number is below the Target
        	this.speed = Math.abs(Speed);
        }
		
		this.targetArm = targetArm;
		ArmCTRL.Motor.stopMotor(); // Stop Motors
	}	
	
	@Override
	public void initialize(){
		
	}
	
	@Override
	public void start(){
		// Do not reset the Encoder Since it is all relative to the Physical Robot.
		
		// Less than the Target
		if(ArmCTRL.LiftEncoder.getDistance() < this.targetArm){
			// Up POS
			this.tooHigh = true; 
			this.speed = Math.abs(this.speed); // Needs to go Up
		}
		
		// Greater than the Target
		else if(ArmCTRL.LiftEncoder.getDistance() > this.targetArm) {
			// Down POS
			this.tooHigh = false; 
			this.speed = -Math.abs(this.speed); // Needs to go Down
		}
	}
	
	@Override
	protected void execute() {
		// TODO: Add in the Lift Encoder Detection and the Light sensor Detection.
		// TODO: Implement the same Saftey feture as the DriveCTRL has line 83.
		// Do the Drive Operation.
		
		if( this.tooHigh ){
			// To High, Needs to go Down.
			if(ArmCTRL.LiftEncoder.get() > this.targetArm){
				// If the Arm is Lower than the Target
				ArmCTRL.Motor.setSpeed(this.speed);
			}
			else {
				ArmCTRL.Motor.setSpeed(0);
				this.isDone = true;
			}
		}
		else {
			// To Low, Needs to go up.
			if(ArmCTRL.LiftEncoder.get() < this.targetArm){
				// If the Arm is Lower than the Target
				ArmCTRL.Motor.setSpeed(this.speed);
			}
			else {
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
