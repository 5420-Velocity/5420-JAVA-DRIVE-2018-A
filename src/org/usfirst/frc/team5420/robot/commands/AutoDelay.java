package org.usfirst.frc.team5420.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj.command.Command;

public class AutoDelay extends Command {

	boolean isDone = false;
	private Date EStopEncoderTime;
	int delayTime = 0;
	
	/**
	 * 
	 * @param mstime Time in Milliseconds to delay the Auto from running on the bot.
	 */
	public AutoDelay (double mstime){
		this.delayTime = (int) mstime; // Cast d as an Int not a Double
	}
	
	
	@Override
	public void initialize(){
		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, (int)this.delayTime); // Time to Check the Encoder Distance is not Zero
		EStopEncoderTime = calculateDate.getTime();
	}
	
	@Override
	protected void execute() {
		// Not the Best but is the working and best solution.
		if( new Date().after(this.EStopEncoderTime) ) {
			this.isDone = true;
		}
	}
	
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
