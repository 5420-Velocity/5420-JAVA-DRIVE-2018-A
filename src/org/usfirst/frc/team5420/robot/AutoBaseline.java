package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class AutoBaseline extends Command {
	
	private boolean isFinished = false;
	public MecDrive DriveControl = null;
	public double VGyroDepend = 0;
	public Gyro gyroSRC = null;
	public Encoder leftEncoder = null;
	public Encoder rightEncoder = null;
	public double EncoderSpaces = 1;
	public String robotPos = Robot.robotPos.getSelected();
	public double baseLine = 1;
	public int step = 0;
	public char[] GamePos = DriverStation.getInstance().getGameSpecificMessage().toCharArray();
	
	public AutoBaseline (MecDrive DriveBase, Gyro GyroIn, Encoder lEncoder, Encoder rEncoder, double EncoderSpaces){
		this.DriveControl = DriveBase;
		this.gyroSRC = GyroIn;
		this.leftEncoder = lEncoder;
		this.rightEncoder = rEncoder;
		this.EncoderSpaces = EncoderSpaces;
		System.out.println("AutoBaseLine");
		this.baseLine = (128*EncoderSpaces);
	}
	
	@Override
	protected void initialize(){
		this.isFinished = false;
	}
	
	@Override
	public void start(){
		
	}
	
	@Override
	protected void execute() {
		// The Switch with the Step int, you are able to move 
		//   to different types of actions dependent on actions.
		System.out.println(this.step);
		switch (this.step){
		
			case 0:
				// Used for Pre-Flight Checks
				next_step();
				resetEncoder();
			break;
		
			// Drive Forward
			case 1:
				// The 140in is to the edge of the Switch, That is from the Wall to the Line!
				if(getDriveEncoder() <= this.baseLine ){ 
					DriveControl.drive(0.4, 0, 0);
				}
				else {
					DriveControl.drive(0, 0, 0);
					this.step++;
				}
			break;
			
			case 999:
			default:
				// If no more cases are left and they are not caught, Finish the Command.
				this.isFinished = true;
			break;
		}
	}
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	/**
	 * This funciton is to Set the Step to 999.
	 *  999 is going to run the Default that will set the isFinished to True.
	 */
	public void next_step(){
		this.step++;
	}
	public void jumpto(int j){
		this.step = j;
	}
	public void end_steps(){
		this.step = 999;
	}
	
	// Custom Functions to help with the Processing of Requests and the Auto.
	public double getGyro(){
		return this.gyroSRC.getAngle();
	}
	public double getVGyro(){
		return ( this.gyroSRC.getAngle() + this.VGyroDepend);
	}
	
	public void zeroGyro(){
		this.gyroSRC.reset();
	}
	public void zeroVGyro(){
		this.VGyroDepend = -getGyro();
	}
	
	
	public int getDriveEncoder(){
		return (int) Math.max(
			leftEncoder.getDistance(),
			rightEncoder.getDistance()
		);
	}
	
	public void resetEncoder(){
		this.leftEncoder.reset();
		this.rightEncoder.reset();
	}
	
	
}