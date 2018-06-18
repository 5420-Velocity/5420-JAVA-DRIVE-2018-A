package org.usfirst.frc.team5420.robot.commands;

import org.usfirst.frc.team5420.robot.MecDrive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Command;

public class TurnCTRL extends Command {

	boolean isDone = false;
	static MecDrive LocalDrive = null;
	static ADXRS450_Gyro Gyro = null;
	static double VirtualOffset = 0;
	double Turn = 0;
	private double degTarget;
	
	
	/**
	 * Setup the Class Static Varables, MecDrive and Gyro
	 * 
	 * @param DRIVEIN       MecDrive Class Passed in, Has Motors and other things defined.
	 * @param gyroSensor    Used for any Turn Commands
	 */
	public TurnCTRL (MecDrive DRIVEIN, ADXRS450_Gyro gyroSensor){
		TurnCTRL.LocalDrive = DRIVEIN;
		TurnCTRL.Gyro = gyroSensor;
	}
	
	/**
	 * Set the Speed and Turn DEG for a Target
	 * 
	 * @param Power   Input for the Power to go Forward.
	 * @param Turn    Input to Drive on a Dime.
	 * @param Crab    Input to Translate Left or Right.
	 * @param TurnDEG Turn untill Input Reached
	 */
	public TurnCTRL (double Speed, int turnDEG){
		
    	// TODO: Check the Turn power is being check when you get the Robot
		if( turnDEG > 0 ){
        	// Number is +
			this.Turn = Math.abs(Speed);
        }
        else if( turnDEG < 0 ){
        	// Number is -
        	this.Turn = -( Math.abs(Speed) );
        }
		
		this.degTarget = turnDEG;
		LocalDrive.stop(); // Stop Motors
	}	
	
	@Override
	public void initialize(){
		zeroVGyro(); // Reset the Gyro to the Virtual Offset.
	}
	
	@Override
	protected void execute() {
		
		// Do the Drive Operation.
		if(Math.abs(getVGyro()) <= Math.abs(this.degTarget) ){ 
			TurnCTRL.LocalDrive.drive(0, this.Turn, 0);
		}
		else {
			TurnCTRL.LocalDrive.drive(0, 0, 0);
			this.isDone = true;
		}
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}
	
	public double getVGyro(){
		return ( TurnCTRL.Gyro.getAngle() + TurnCTRL.VirtualOffset );
	}
	public void zeroVGyro(){
		TurnCTRL.VirtualOffset = -TurnCTRL.Gyro.getAngle(); // Negative the Number to Subtract or add the Current Value.
	}

}
