/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5420.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team5420.robot.MecDrive;
import org.usfirst.frc.team5420.robot.OI;
import org.usfirst.frc.team5420.robot.commands.ExampleCommand;
import org.usfirst.frc.team5420.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	public static final String ScaleAuto = "DoScaleAuto";
	public static final String CenterSwitchAuto = "CenterAuto";
	public static final String SwitchLeft = "LeftAuto";
	public static final String SwitchRight = "RightAuto";
	public static final String NoAuto = "NoAuto";
	public static final String BaseLine = "baseLine";
	
	public static final int ROBOT_POS_ONE = 1; // Left
	public static final String ROBOT_POS_ONE_STR = "Left"; // Left
	public static final int ROBOT_POS_TWO = 2; // Center
	public static final int ROBOT_POS_CENTER = 2; // Center
	public static final String ROBOT_POS_TWO_STR = "Center"; // Left
	public static final int ROBOT_POS_THREE = 3; // Right
	public static final String ROBOT_POS_THREE_STR = "Right"; // Right
	
	// Using the 500 per 4FT
	public static final int EncoderFT = 125;
	public static final int EncoderIN = EncoderFT/12;
	
	public static final double MaxHightEncoder = -7000;
	public double VirtualOffset = 0;
	
	public DriverStation.Alliance color;
	public double time;
	public static OI jio;
	String autonomousCommand;
	//SendableChooser<Command> chooser = new SendableChooser<>();
	SendableChooser<String> chooser = new SendableChooser<>();
	SendableChooser<String> robotPos = new SendableChooser<>();
	
	// Setup of the Devices in the Code.
	Timer timer = new Timer();
	public ADXRS450_Gyro gyroSensor;
	public UsbCamera camera;
	
	public static Compressor compressor0;
	public static  Solenoid solenoid0, solenoid1;
	public static  Solenoid breakOn, breakOff;
	
	public static Ultrasonic liftSensor; 
	public static boolean AutoDelayFinished = false;
	
	public static Encoder encoder0, encoder1;
	public static Encoder encoderArm, encoderLift;
	public static DigitalInput upperLimit, lowerLimit;
	public static DigitalInput CloseMiss;
	
	public static Joystick joystick0, joystick1;
	public static VictorSP LiftMotor, ArmMotor;
	
	// Motor Setup
	public VictorSP motorFL, motorRL, motorRR, motorFR;
	public MecDrive MyDrive;
	
	// The Preferences for the Robot Preferences Pannel in the SmartDashboard
	public Preferences SmartDashboardPref;
	// @link https://wpilib.screenstepslive.com/s/currentCS/m/java/l/219414-power-distribution-panel
	PowerDistributionPanel pdp;
	
	public char[] GamePos;
	
	/**
	 * When it is true the Claw is Closed, when it is False It should be Open.
	 */
	public boolean clawState = true; // Default, Assuming the Close State for Auto

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putBoolean("AutoInitComplete", false);
		SmartDashboard.putBoolean("TeleopInitComplete", false);
		SmartDashboard.putBoolean("RobotInit", false);
		
		SmartDashboard.putNumber("DrivePower", 0 ); // Init Drive Station Values
		SmartDashboard.putNumber("DriveTurn", 0 ); // Init Drive Station Values
		SmartDashboard.putNumber("DriveCrab", 0 ); // Init Drive Station Values
		SmartDashboard.putNumber("Battery", 0); // Init Drive Station Values
		SmartDashboard.putNumber("Angle", 0 );
		SmartDashboard.putNumber("VAngle", 0 );
		SmartDashboard.putBoolean("ClawOpenSignal", clawState);
		SmartDashboard.putBoolean("CloseMiss", false);
		SmartDashboard.putBoolean("ResetDriveENC", false);
		SmartDashboard.putBoolean("UpperLimit", false);
		SmartDashboard.putBoolean("LowerLimits", false);
		SmartDashboard.putString("AutoSelect", "(Not Set)");
		
		SmartDashboard.putNumber("AutoDelay", 0); // Init Drive Station Values
		
		//chooser.addObject(name, object);
		chooser.addDefault("No Auto Selected", NoAuto);
		chooser.addObject("Center Auto", CenterSwitchAuto);
		chooser.addObject("Scale Auto", ScaleAuto);
		chooser.addObject("Left Switch Auto", SwitchLeft);
		chooser.addObject("Right Switch Auto", SwitchRight);
		chooser.addObject("Base Line Auto", BaseLine);
		SmartDashboard.putData("AutoChoices", chooser);
				
		robotPos.addObject("Left (1)", ROBOT_POS_ONE_STR);
		robotPos.addDefault("Center (2)", ROBOT_POS_TWO_STR);
		robotPos.addObject("Right (3)", ROBOT_POS_THREE_STR);
		SmartDashboard.putData("Position", robotPos);
		
		camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setExposureAuto();
		camera.setResolution(640, 480);
		
		
		color = DriverStation.getInstance().getAlliance();
		time = DriverStation.getInstance().getMatchTime();
		
		jio = new OI();
		
		gyroSensor = new ADXRS450_Gyro( SPI.Port.kOnboardCS0 ); // SPI
		gyroSensor.calibrate(); // SPI
		
		compressor0 = new Compressor(0);
		solenoid0 = new Solenoid(1); // Claw Close 
		solenoid1 = new Solenoid(2); // Claw Open
		breakOn = new Solenoid(3); // Pull the Cylinder Close, Break on 
		breakOff = new Solenoid(4); // Push the Cylinder Open, Break off
		
		liftSensor = new Ultrasonic(2,3); // creates the ultra object andassigns ultra to be an ultrasonic sensor which uses DigitalOutput 1 for 
		
		encoder0 = new Encoder(4,5, true, Encoder.EncodingType.k4X); // Left DIO, DIO, Reversed Count Direction since it is the Right Side
		encoder1 = new Encoder(6,7, false, Encoder.EncodingType.k4X); // Right DIO, DIO
		encoderLift = new Encoder(10,11, false, Encoder.EncodingType.k4X); // Lift DIO, DIO
		encoderArm = new Encoder(12,13, true, Encoder.EncodingType.k4X); // Arm DIO, DIO
		
		joystick0 = new Joystick(0); //Controller One USB
		joystick1 = new Joystick(1); //Controller Two USB
		
		lowerLimit = new DigitalInput(17); // DIO
		//UpperLimit = new DigitalInput(1); // DIO
		CloseMiss = new DigitalInput(25); // Disableed, Used the same Interface as the Encoder
		
		LiftMotor = new VictorSP(2); // PWM
		ArmMotor = new VictorSP(1); // PWM
		
		motorFL = new VictorSP(3); // PWM
		motorRL = new VictorSP(4); // PWM
		motorFR = new VictorSP(6); // PWM
		motorRR = new VictorSP(5); // PWM
		
		//       frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor
		MyDrive = new MecDrive(motorFL, motorFR, motorRL, motorRR);
		//MyDrive.setGyro(gyroSensor); // Send the Gyro Object
		MyDrive.invert(true);
		MyDrive.setDeadband(0.12);
		MyDrive.enableDeadband();
		
		pdp = new PowerDistributionPanel();
		SmartDashboard.putBoolean("RobotInit", true);
	}
	
	/**
	 * Periodic code for all robot modes should go here.
	 * @link http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/IterativeRobotBase.html#robotPeriodic--
	 */
	@Override
	public void robotPeriodic(){
		
		heartbeat();
		SmartDashboard.putNumber("Angle", getGyro() );
		SmartDashboard.putNumber("VAngle", getVGyro() );
		
		SmartDashboard.putNumber("LiftPos", encoderLift.getDistance());
		SmartDashboard.putNumber("LiftArmPos", encoderArm.getDistance());
		
		SmartDashboard.putNumber("EncoderLeft", encoder0.getDistance());
		SmartDashboard.putNumber("EncoderRight", encoder1.getDistance());
		SmartDashboard.putBoolean("UpperLimit", false);
		SmartDashboard.putBoolean("LowerLimits", lowerLimit.get());
		
		SmartDashboard.putNumber("Battery", pdp.getVoltage());
		SmartDashboard.putNumber("CurrentAmps", pdp.getTotalCurrent());
		
		SmartDashboard.putBoolean("CloseMiss", CloseMiss.get());
		
		// Reset the Encoder Values
		if(SmartDashboard.getBoolean("ResetDriveENC", false) == true){
			SmartDashboard.putBoolean("ResetDriveENC", false);
			encoder0.reset();
			encoder1.reset();
			
			encoderLift.reset();
			encoderArm.reset();
		}
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		timer.stop();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		int baseLine = (128*EncoderIN);
		solenoidUpdate(clawState, solenoid0, solenoid1); // Set the init State, Set asap.
		SmartDashboard.putBoolean("ClawOpenSignal", clawState);
		
		// Auto Timer Delay
		if( SmartDashboard.getNumber("AutoDelay", 0) != 0 ){
			// Get the Timer Delay from the Dashboard and Delay the Auto.
			Timer.delay( SmartDashboard.getNumber("AutoDelay", 0) );
		}
		
		timer.reset();
		timer.start();
		compressor0.setClosedLoopControl(true);
		autonomousCommand = this.chooser.getSelected();
		SmartDashboard.putString("AutoSelect", autonomousCommand);
		String autoSelect = autonomousCommand;
		String robotPos = this.robotPos.getSelected();
		
		this.GamePos = DriverStation.getInstance().getGameSpecificMessage().toCharArray();
		/**
		 * DriverStation.getInstance().getGameSpecificMessage() returns the Data of the Left or Right 
		 *      Direction of your Color on the Switch Scale Switch.
		 * The First Char of the String will ALWAYS be the once closest to your drive station wall.
		 * 
		 * if( GamePos[0] == 'L' ) // The Close Switch Left Side is your Color.
		 * if( GamePos[1] == 'L' ) // Scale, Center Field, Left Side
		 * if( GamePos[2] == 'L' ) // The Far Switch Left Side is your Color.
		 */
		
		if(autonomousCommand == NoAuto){
			System.out.println("No Auto Selected!");
			encoder0.reset();
		}
		
		// LR, Auto for the Scale
		else if(autoSelect == ScaleAuto){
			// Be sure to Check the Color of the Scale or Switch before running and the Placement.
		}
		
		// C, Swtich
		else if(autoSelect == CenterSwitchAuto){
			// Be sure to Check the Color of the Scale or Switch before running and the Placement.
			if(robotPos == ROBOT_POS_THREE_STR){
				
			}
			
		}
		
		// L Swtich
		else if(autoSelect == SwitchLeft){
			
			// Be sure to Check the Color of the Scale or Switch before running and the Placement.
			if(GamePos[1] == 'L' && robotPos == ROBOT_POS_ONE_STR){
				// If the Color is on the Left Side and we are in POS 1
				
				// Drive past the Base
				resetEncoder();
				while(true){
					// The 140in is to the edge of the Switch, That is from the Wall to the Line!
					if(getDriveEncoder() <= baseLine ){ 
						MyDrive.drive(0.4, 0, 0);
					}
					else {
						MyDrive.drive(0, 0, 0);
						break;
					}
				}
				
				// Turn Left to 45deg
				zeroVGyro();
				while(true){
					// The 140in is to the edge of the Switch, That is from the Wall to the Line!
					if(getVGyro() <= 45 ){ 
						MyDrive.drive(0, 0.4, 0);
					}
					else {
						MyDrive.drive(0, 0, 0);
						break;
					}
				}
				zeroVGyro();
				
			}
			
		}
		
		// R Swtich
		else if(autoSelect == SwitchRight){
			// Be sure to Check the Color of the Scale or Switch before running and the Placement.
			
			if(GamePos[1] == 'R' && robotPos == ROBOT_POS_THREE_STR){
				// If the Color is on the Right Side and we are in POS 3.
				
			}
			
			
		}
		
		// LR, Drive Past the Base Line
		else if(autoSelect == BaseLine){	
			// Conver this section of the Code for the Auto into 
			//  Seperate Command Class for Auto Chooser Object Selection.
			while(true){
				// The 140in is to the edge of the Switch, That is from the Wall to the Line!
				if(getDriveEncoder() <= baseLine ){ 
					MyDrive.drive(0.5, 0, 0);
				}
				else {
					MyDrive.drive(0, 0, 0);
					break;
				}
			}
			
		}
		
		// Schedule the autonomous command
		if (autonomousCommand != null)
			//autonomousCommand.start();
		SmartDashboard.putBoolean("AutoInitComplete", true);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		this.MyDrive.DriveControl.setSafetyEnabled(true); // Shuts off motors when their outputs aren't updated often enough.
		
		if (autonomousCommand != null)
			//autonomousCommand.cancel();
		SmartDashboard.putBoolean("TeleopInitComplete", true);
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		// Driver B Button (XBOX)
		// This Sets all of the Motors to a Stop State when Pressed.
			double powerJoy = ( (-joystick0.getRawAxis(2)) + joystick0.getRawAxis(3) ); // Add the 2 values from the Controller Inputs
			double turnJoy = joystick0.getRawAxis(0);
			double crabJoy = joystick0.getRawAxis(4);
			
			if(joystick0.getRawButton(2) || joystick1.getRawButton(2) ){
				MyDrive.DriveControl.stopMotor();
			}
			else {
				MyDrive.drive(powerJoy, turnJoy, crabJoy);
			}
		
			SmartDashboard.putNumber("DrivePower", powerJoy ); // Send Value to the Dashboard
			SmartDashboard.putNumber("DriveTurn", turnJoy ); // Send Value to the Dashboard
			SmartDashboard.putNumber("DriveCrab", crabJoy ); // Send Value to the Dashboard
		
		// Operator A Button (XBOX)
		// This is to open and close the Solenoid
			if(joystick1.getRawButton(1) || joystick0.getRawButton(1)){
				clawState = false;
				SmartDashboard.putBoolean("ClawOpenSignal", clawState);
				solenoidUpdate(clawState, solenoid0, solenoid1); // Set the State
			}
			else {
				clawState = true;
				SmartDashboard.putBoolean("ClawOpenSignal", clawState);
				solenoidUpdate(clawState, solenoid0, solenoid1); // Set the State
			}
		
		// Operator
		// This is to control the Lift action and it's motor+break.
			if(joystick1.getRawButton(3)){
				// Up
				System.out.println("UP");
				LiftMotor.setSpeed(0.9);
			}
			else if(joystick1.getRawButton(4)) {
				// Down
				System.out.println("DOWN");
				LiftMotor.setSpeed(-0.6);
			}
			else {
				LiftMotor.stopMotor();
			}

		// Operator
		// This is for the Arm Lift Control
			double ArmValue = joystick1.getRawAxis(1);
			System.out.println("L: "+ArmValue);
			if(ArmValue > 0.4){
				// Up
				if(encoderLift.getDistance() <= MaxHightEncoder) {
					// Past Max Height
					System.out.println("L: Max Height");
					ArmMotor.setSpeed(0);
				}
				else {
					// UP
					System.out.println("L: Up");
					solenoidUpdate(false, breakOn, breakOff);
					ArmMotor.setSpeed(0.7);
				}
			}
			else if(ArmValue < -0.4) {
				// Down
				// Keep in mind the Lower Limit Switch is Always True till the Arm Comes down and interupts the Light, Makes it False.
				if(lowerLimit.get()){
					System.out.println("L: Down");
					// Light is on by Default (Reflecting and getting Light Normal, When the Arm is not down.)
					solenoidUpdate(false, breakOn, breakOff);
					ArmMotor.setSpeed(-0.7);
				}
				else {
					// Auto Limit and Reset for Calibration of the Lift Sensor
					encoderLift.reset();  // Zero Sonsor
					System.out.println("L: Dower -- Lower Limit Reached.");
				}
			}
			else {
				solenoidUpdate(true, breakOn, breakOff);
				System.out.println("L: Normal");
				ArmMotor.stopMotor();
			}
			
		Scheduler.getInstance().run();
	}
	
	
	
	public double getGyro(){
		return this.gyroSensor.getAngle();
	}
	public double getVGyro(){
		return ( this.gyroSensor.getAngle() + this.VirtualOffset);
	}
	
	public void zeroGyro(){
		this.gyroSensor.reset();
	}
	public void zeroVGyro(){
		this.VirtualOffset = -getGyro();
	}
	
	
	public int getDriveEncoder(){
		return (int) Math.max(
			Robot.encoder0.getDistance(),
			Robot.encoder1.getDistance()
		);
	}
	
	public void resetEncoder(){
		Robot.encoder0.reset();
		Robot.encoder1.reset();
	}
	
	/**
	 * This Smartdashboard 'RunningHeartbeat' is updating the Runtime value of the Robot.
	 * Used in the dashboard to see if the robot is running. Send the Unix Time of the RIO.
	 * @link https://stackoverflow.com/a/732043/5779200
	 */
	public void heartbeat(){
		time = DriverStation.getInstance().getMatchTime();
		SmartDashboard.putNumber("MatchTime", time);
		SmartDashboard.putNumber("SystemTime", (int) System.currentTimeMillis()/1000 );
		SmartDashboard.putNumber("RunningHeartbeat", timer.get());
	}
	
	/**
	 * Used to set the State for the Solinoid Controls
	 * @param NewState NewState The State to set on the solinoids
	 * @param Port1 Solenoid When State is True it is Set to True
	 * @param Port2 Port2 When State is True it is Set to False 
	 */
	public void solenoidUpdate(boolean NewState, Solenoid Port1, Solenoid Port2){
		Port1.set(NewState);
		Port2.set(!NewState);
	}
}
