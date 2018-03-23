package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class MecDrive {
	public double deadband = 0;
	public boolean deadband_enabled = false;
	private SpeedController leftFront, leftRear, rightFront, rightRear;
	public MecanumDrive DriveControl;
	public Gyro gyroSensor = null;

	/*
	 * Mecanum Drive Init
	 * 
	 * @param SpeedController Left Front Controller
	 * @param SpeedController Right Front Controller
	 * @param SpeedController Left Rear Controller
	 * @param SpeedController Right Rear Controller
	 */
	//               SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor, SpeedController rearRightMotor
	public MecDrive( SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor, SpeedController rearRightMotor ){
		this.leftFront = frontLeftMotor;
		this.leftRear = rearLeftMotor;
		this.rightFront = frontRightMotor;
		this.rightRear = rearRightMotor;
		
		DriveControl = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor); // Start the WPI Motor Control
		DriveControl.setSafetyEnabled(false); // MecanumDrive > RobotDriveBase > MotorSafety: Shuts off motors when their outputs aren't updated often enough.
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
	public MecDrive( SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor, SpeedController rearRightMotor, double deadband ){
		this(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
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
		//                        ^  ^  ^ ^
		//                        |  |  | |
		//       Left / Right    /   |  | |
		//                Rotation  /   | |
		//            Forward Reverse  /  |
		//                         Gyro  /
		//
		Power = this.deadzone(Power);
		Turn = this.deadzone(Turn);
		Crab = this.deadzone(Crab);
		
		if (this.gyroSensor == null){
			this.DriveControl.driveCartesian(Power, Crab, Turn, 0);
		}
		else {
			this.DriveControl.driveCartesian(Power, Crab, Turn, this.gyroSensor.getAngle());
			System.out.println("Gyro");
		}
	}
	
	public void setGyro(Gyro gyroIn){
		this.gyroSensor = gyroIn;
	}
	
	public void invert(boolean doInvert){
		this.leftFront.setInverted(doInvert);
		this.leftRear.setInverted(doInvert);
		this.rightFront.setInverted(doInvert);
		this.rightRear.setInverted(doInvert);
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
	 * @param double Input of gray Zone
	 */
	public void setDeadband (double limit) {
		this.deadband = limit;
	}
	
	/**
	 * Stop the Actions for Applying restrictions on the deadband.
	 * @param double The Value to apply the Deadband to.
	 */
	public void disableDeadband() {
		this.deadband_enabled = false;
	}
	
	/**
	 * Apply the deadband restriction to the value input using the stored value.
	 * @param double The Value to apply the Deadband to.
	 * @return double Returns the Deadband value.
	 */
	public void enableDeadband() {
		this.deadband_enabled = false;
	}
	
	/**
	 * Apply the deadband on the Input Value.
	 * @param joystick The Controller Input.
	 * @param deadband The Limit Zone for the Controller Input.
	 * @return The Final Value (Double)
	 */
	public double deadzone(double joystick, double deadband){
    	if( Math.abs(joystick) < deadband ){
    		return 0;
    	}
    	else {
    		return joystick;
    	}
    }
	
	/**
	 * Applies the Deadband based off of the System Value
	 * @see deadzone
	 * @param joystick
	 * @return
	 */
	public double deadzone(double joystick){
    	if( Math.abs(joystick) < this.deadband ){
    		return 0;
    	}
    	else {
    		return joystick;
    	}
    }
	
}
