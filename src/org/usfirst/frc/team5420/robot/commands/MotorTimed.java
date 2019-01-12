package org.usfirst.frc.team5420.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import org.usfirst.frc.team5420.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.command.Command;

public class MotorTimed extends Command {

	boolean isDone = false;
	private PWMSpeedController Motor = null;
	int targetTime;
	double speed = 0.2;
	private boolean tooHigh = false;
	private Date EStopEncoderTime;
	
	/**
	 * Action INIT
	 * 
	 * @param Speed      Speed used to drive the motor
	 * @param targetTime  The Time to wait before turning off the motors.
	 */
	public MotorTimed (PWMSpeedController MOTORIN, double Speed, int targetTimeIn){
		this.targetTime = Math.abs(targetTimeIn); // Get ABS of the Number.
		this.speed = Speed;
		this.Motor = MOTORIN;
	}	
	
	@Override
	public void initialize(){
		// Setup the Stop Motor by Time.
		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.SECOND, (int) this.targetTime); // Time to Check the Encoder Distance is not Zero
		EStopEncoderTime = calculateDate.getTime();
	}
	
	@Override
	protected void execute() {
		// TODO: Add in the Lift Encoder Detection and the Light sensor Detection.
		// TODO: Implement the same Safety feature as the DriveCTRL has line 83.
		// Do the Drive Operation.
		
		if( this.tooHigh ){
			this.Motor.set(0);
			this.isDone = true;
		}
		else {
			this.Motor.set(this.speed);
		}
		
		
		// Safety Code, Its made to catch the Human Error of not plugging in the Encoder.
		//  Encoders will send a 0 value if you don't have an encoder plugged-in to the port.
		// Do the Safe Check to see if the Encoders are doing their thing or not after x seconds
		if( new Date().after(EStopEncoderTime) ) {
			// If the Encoder is not Past 10 ticks.
			System.out.println("Reached Target Time. [MotorTimed]");
			this.Motor.setSpeed(0);
			this.isDone = true;
		}
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}
