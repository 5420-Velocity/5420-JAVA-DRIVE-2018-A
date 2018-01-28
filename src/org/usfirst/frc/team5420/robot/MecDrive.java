package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class MecDrive {
	public double deadband = 0;
	public boolean deadband_enabled = false;
	private SpeedController leftFront, rightFront, leftRear, rightRear;
	private MecanumDrive DriveControl;

	/*
	 * Mecanum Drive Init
	 * 
	 * @param SpeedController Left Front Controller
	 * @param SpeedController Right Front Controller
	 * @param SpeedController Left Rear Controller
	 * @param SpeedController Right Rear Controller
	 */
	public MecDrive( SpeedController inLF, SpeedController inRF, SpeedController inLR, SpeedController inRR ){
		this.leftFront = inLF;
		this.rightFront = inRF;
		this.leftRear = inLR;
		this.rightRear = inRR;
		
		DriveControl = new MecanumDrive(inLF, inLR, inRF, inRR); // Start the WPI Motor Control
	}
	
	/*
	 * Mecanum Drive Init with Deadband Enabled
	 * 
	 * @param SpeedController Left Front Controller
	 * @param SpeedController Right Front Controller
	 * @param SpeedController Left Rear Controller
	 * @param SpeedController Right Rear Controller
	 * @param double		  Deadband Value
	 */
	public MecDrive( SpeedController inLF, SpeedController inRF, SpeedController inLR, SpeedController inRR, double deadband ){
		this(inLF, inRF, inLR, inRR);
		this.deadband = deadband;
		this.deadband_enabled = true;
	}
	
	public void stop() {
		this.leftFront.set(0);
		this.rightFront.set(0);
		this.leftRear.set(0);
		this.rightRear.set(0);
	}
	
	/**
	 * 
	 * @param en Boolean Enable or Disable the Deadband 
	 */
	public void deadband(boolean en){
		this.deadband_enabled = en;
	}
	public void zeroDeadband(){
		this.deadband = 0;
	}
	
	/**
	 * The Drive Control that tells the WPI Loop what to drive.
	 * @param double Forward/Reverse
	 * @param double Value for Forward/Reverse and Left/Right
	 * @param double Left/Right
	 */
	public void drive( double Power, double Turn, double Crab ){
		//myDrive.driveCartesian(0, 0, 0, 0);
		//                        ^  ^  ^
		//                        |  |  |
		//       Left / Right    /   |  |
		//                Rotation  /   |
		//            Forward Reverse  /
		//
		Power = do_deadband(Power);
		Turn = do_deadband(Turn);
		Crab = do_deadband(Crab);
		this.DriveControl.driveCartesian(Crab, Turn, Power, 0);
	}
	
	/**
	 * Move the Robot Forward
	 * @see drive
	 */
	public void forward(double Speed){
		drive( Math.abs(Speed), 0, 0);
	}
	
	/**
	 * Move the Robot Reverse (Back)
	 * @see drive
	 */
	public void reverse(double Speed){
		drive( -Math.abs(Speed) , 0, 0);
	}
	
	/**
	 * Move the Robot Left ( <-- )
	 * @see drive
	 */
	public void left(double Speed){
		drive(0, 0, Math.abs(Speed) );
	}
	
	/**
	 * Move the Robot Right ( --> )
	 * @see drive
	 */
	public void right(double Speed){
		drive(0, 0, -Math.abs(Speed) );
	}
	
	/**
	 * Move the Robot Diagonal
	 * @param double Speed Direction [Forward/Backward]
	 * @param char   Direciton [Left/Right]
	 * @see drive
	 */
	public void diagonal(double Speed, char Direction){
		double CrabSpeed = Speed;
		if(Direction == new Character('L')){
			// If the Direction is Left
			CrabSpeed = Math.abs(Speed);
		}
		else if(Direction == new Character('R')) {
			// If the Direction is Right
			CrabSpeed = Math.abs(Speed)*-1;
		}
		drive(Speed, 0, CrabSpeed);
	}
	
	/**
	 * Move the Robot Crab
	 * @param double Speed
	 * @param char   Direciton [Left/Right]
	 * @see drive
	 */
	public void crab(double Speed, char Direction){
		double CrabSpeed = Speed;
		if(Direction == new Character('L')){
			// If the Direction is Left
			CrabSpeed = Math.abs(Speed);
		}
		else if(Direction == new Character('R')) {
			// If the Direction is Right
			CrabSpeed = Math.abs(Speed)*-1;
		}
		drive(0, 0, CrabSpeed);
	}
	
	/**
	 * Move the Robot Crab Right
	 * @see crab
	 */
	public void crab_right(double Speed){
		crab(Speed, 'R');
	}
	
	/**
	 * Move the Robot Crab Left
	 * @see crab
	 */
	public void crab_left(double Speed){
		crab(Speed, 'L');
	}
	
	/**
	 * If the Deadband is enabled in the Class then apply the deadband.
	 * @param double Joystick in Value.
	 * @return double The Joystick Value.
	 */
	public double do_deadband (double joystick) {
		if(this.deadband_enabled == true){
			return this.deadband(joystick);
		}
		else {
			return joystick;
		}
	}
	
	/**
	 * Apply the deadband restriction to the value input using the stored value.
	 * @param double The Value to apply the Deadband to.
	 * @return double Returns the Deadband value.
	 */
	public double deadband(double joystick) {
	    if(joystick < this.deadband || (joystick)*-1 > this.deadband ) return 0;
	    else return joystick;
	}
	
	/**
	 * Apply the deadband restriction to the value input using the given Input.
	 * @param double Joystick in Value.
	 * @param double The zone for the deadband to ingore (The Dead zone).
	 * @return
	 */
	public double deadband(double joystick, double dead) {
	    if(joystick < dead || (joystick)*1 > dead) return 0;
	    else return joystick;
	}
	
}
