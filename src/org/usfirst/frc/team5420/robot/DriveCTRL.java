package org.usfirst.frc.team5420.robot;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCTRL extends Command {

	boolean isDone = false;
	static MecDrive LocalDrive = null;
	static Encoder EncoderLEFT = null;
	static Encoder EncoderRIGHT = null;
	double VirtualOffset = 0; // Does not get set globally since it is per instance Value
	double Power = 0;
	double Turn = 0;
	double Crab = 0;
	int DriveDistance = 2;
	private Date EStopEncoderTime;
	
	
	/**
	 * Setup the Static Varables in the Class for the Drive System, Encoder Left and Right.
	 * 
	 * @param DRIVEIN       MecDrive Class Passed in, Has Motors and other things defined.
	 * @param gyroSensor    Used for any Turn Commands
	 * @param EncoderLEFT   
	 * @param EncoderRIGHT  
	 * @param encoderin     
	 */
	public DriveCTRL (MecDrive DRIVEIN, Encoder EncoderLEFT, Encoder EncoderRIGHT){
		DriveCTRL.LocalDrive = DRIVEIN;
		DriveCTRL.EncoderLEFT =  EncoderLEFT;
		DriveCTRL.EncoderRIGHT = EncoderRIGHT;
		LocalDrive.stop();
	}
	
	/**
	 * Run the Instance for the System to run.
	 * 
	 * @param Power Input for the Power to go Forward.
	 * @param Turn  Input to Drive on a Dime.
	 * @param Crab  Input to Translate Left or Right.
	 */
	public DriveCTRL (double Power, double Turn, double Crab, int Distance){
		this.Power = Power;
		this.Turn = Turn;
		this.Crab = Crab;
		this.DriveDistance = Distance;
		LocalDrive.stop();
	}
	
	@Override
	public void initialize(){
		
	}
	
	@Override
	public void start(){
		// Setup the Safety Time.
			Calendar calculateDate = GregorianCalendar.getInstance();
			calculateDate.add(GregorianCalendar.SECOND, 2); // Time to Check the Encoder Distance is not Zero
			EStopEncoderTime = calculateDate.getTime();
		
		resetEncoder();
	}
	
	@Override
	protected void execute() {
		
		// Do the Drive Operation.
		if(getDriveEncoder() <= this.DriveDistance ){ 
			DriveCTRL.LocalDrive.drive(this.Power, this.Turn, this.Crab);
		}
		else {
			DriveCTRL.LocalDrive.drive(0, 0, 0);
			this.isDone = true;
		}
		
		
		// Safety Code, Its made to catch the Human Error of not plugging in the Encoder.
		//  Enocders will send a 0 value if you don't have an encoder plugged-in to the port.
		// Do the Safe Check to see if the Encoders are doing their thing or not after x seconds
		if( new Date().after(EStopEncoderTime) ) {
			// If the Encoder is not Past 10 ticks.
			if( getDriveEncoder() <= 10 ){
				System.err.println("WARN:    Encoder Timeout");
				DriveCTRL.LocalDrive.drive(0, 0, 0);
				this.isDone = true;
			}
		}
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}
	
	// Get Both Encoders and Read to get the Max Encoder Value from them.
	public int getDriveEncoder(){
		return (int) Math.max(
			DriveCTRL.EncoderLEFT.getDistance(),
			DriveCTRL.EncoderRIGHT.getDistance()
		);
	}
	
	public void resetEncoder(){
		DriveCTRL.EncoderLEFT.reset();
		DriveCTRL.EncoderRIGHT.reset();
	}

}
