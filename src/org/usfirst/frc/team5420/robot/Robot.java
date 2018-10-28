/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5420.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5420.robot.MecDrive;
import org.usfirst.frc.team5420.robot.OI;
import org.usfirst.frc.team5420.robot.commands.ArmCTRL;
import org.usfirst.frc.team5420.robot.commands.ArmCTRLHigh;
import org.usfirst.frc.team5420.robot.commands.ArmTimed;
import org.usfirst.frc.team5420.robot.commands.DistanceAlign;
import org.usfirst.frc.team5420.robot.commands.DriveCTRL;
import org.usfirst.frc.team5420.robot.commands.LiftCTRL;
import org.usfirst.frc.team5420.robot.commands.MotorTimed;
import org.usfirst.frc.team5420.robot.commands.SolenoidCTRL;
import org.usfirst.frc.team5420.robot.commands.TurnCTRL;
import org.usfirst.frc.team5420.robot.commands.WaitCTRL;
import org.usfirst.frc.team5420.robot.subsystems.CameraLines;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	public static final String ScaleAuto = "DoScaleAuto";
	public static final String SwitchAuto = "SwitchAuto";
	public static final String NoAuto = "NoAuto";
	public static final String BaseLine = "baseLine";
	public static final String ScaleSwitchAuto = "ScaleSwitchAuto";
	
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
	public SendableChooser<String> chooser = new SendableChooser<>(); // Auto Select
	public SendableChooser<String> robotPos = new SendableChooser<>(); // Position on the Field
	
	// Setup of the Devices in the Code.
	Timer timer = new Timer();
	public ADXRS450_Gyro gyroSensor;
	
	public static UsbCamera camera;
	//public static CameraLines linesCam = null;
	public static int width = 640; // Camera RESOLUTION
	public static int height = 480; // Camera RESOLUTION
	
	public static Compressor compressor0;
	public static Solenoid liftBreakOn, liftBreakOff, armBreakOn, armBreakOff, clawOpen, clawClose;
	public static SolenoidMap armBreak, liftBreak, clawPinch;
	public static SolenoidMap liftBreakMap, armBreakMap;
	
	public static Ultrasonic distSensor; 
	public static boolean AutoDelayFinished = false;
	
	public static Encoder leftDriveEncoder, rightDriveEncoder;
	public static Encoder encoderLift, encoderArm;
	// The EncoderMap has controls for Virtual Offsets, Needed since it Starts Mid Mast.
	public static EncoderMap encoderArmMap; // Encoder Map Class, Has offset and the Get Controls. 
	public static DigitalInput upperLimit, lowerLimit;
	public static DigitalInput flipperUp, flipperDown;
	
	public static Joystick controllerDriver, controllerOperator;
	public static VictorSP LiftMotor, ArmMotor, PivotMotor, IntakeMotor;
	
	// Motor Setup
	public VictorSP motorFL, motorRL, motorRR, motorFR;
	public MecDrive MyDrive;
	
	// The Preferences for the Robot Preferences Panel in the SmartDashboard
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
		//chooser.addObject(name, object);
		chooser.addObject("Switch Auto", SwitchAuto);
		chooser.addObject("No Auto Selected", NoAuto);
		chooser.addObject("Scale Auto", ScaleAuto);
		chooser.addObject("Scale then Switch", ScaleSwitchAuto);
		chooser.addObject("Base Line", BaseLine);
		//chooser.addObject("Do it!!!!", "WORK");
		SmartDashboard.putData("AutoChoices", chooser);
		
		SmartDashboard.putString("Chooser String", chooser.toString());
		
		SmartDashboard.putBoolean("AutoInitComplete", false);
		SmartDashboard.putBoolean("TeleopInitComplete", false);
		SmartDashboard.putBoolean("RobotInit", false);
		
		SmartDashboard.putNumber("DrivePower", 0 ); // Init Drive Station Values
		SmartDashboard.putNumber("DriveTurn", 0 ); // Init Drive Station Values
		SmartDashboard.putNumber("DriveCrab", 0 ); // Init Drive Station Values
		SmartDashboard.putBoolean("LimitedPower", false);
		
		SmartDashboard.putNumber("Battery", 0); // Init Drive Station Values
		SmartDashboard.putNumber("Angle", 0 );
		SmartDashboard.putNumber("VAngle", 0 );
		SmartDashboard.putString("CubeIntake", "Idle"); // Cube Intake State
		SmartDashboard.putBoolean("ResetDriveENC", false);
		SmartDashboard.putBoolean("UpperLimit", false);
		SmartDashboard.putBoolean("LowerLimits", false);
		SmartDashboard.putBoolean("ResendAutoCommands", false);
		//SmartDashboard.putString("AutoSelect", "(Not Set)");
		
		SmartDashboard.putNumber("AutoDelay", 0); // Init Drive Station Values
		
		robotPos.addDefault("Left (1)", ROBOT_POS_ONE_STR);
		robotPos.addObject("Center (2)", ROBOT_POS_TWO_STR);
		robotPos.addObject("Right (3)", ROBOT_POS_THREE_STR);
		SmartDashboard.putData("Position", robotPos);
		
		camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setExposureAuto();
		camera.setResolution(Robot.width, Robot.height);
		//linesCam = new CameraLines(); // Create Camera
		
		color = DriverStation.getInstance().getAlliance();
		time = DriverStation.getInstance().getMatchTime();
		
		jio = new OI(); // Not Used Yet
		
		gyroSensor = new ADXRS450_Gyro( SPI.Port.kOnboardCS0 ); // SPI
		gyroSensor.calibrate(); // SPI
		
		compressor0 = new Compressor(0);
		armBreakOn = new Solenoid(1);
		armBreakOff = new Solenoid(2);
		liftBreakOn = new Solenoid(3); 
		liftBreakOff = new Solenoid(4);
		clawOpen = new Solenoid(5);
		clawClose = new Solenoid(6);
		
		armBreak = new SolenoidMap( armBreakOn, armBreakOff ); // The Lift Break Solenoids
		liftBreak =  new SolenoidMap(liftBreakOn, liftBreakOff); // The Arm Break Solenoids
		clawPinch =  new SolenoidMap(clawOpen, clawClose); // The Arm Break Solenoids
		
		distSensor = new Ultrasonic(0,1); // creates the ultrasonic object and assigns ultrasonic to be an ultrasonic sensor which uses DigitalOutput 1 for 
		
		leftDriveEncoder = new Encoder(4,5, true, Encoder.EncodingType.k4X); // Left DIO, DIO, Reversed Count Direction since it is the Right Side
		rightDriveEncoder = new Encoder(6,7, false, Encoder.EncodingType.k4X); // Right DIO, DIO
		encoderLift = new Encoder(10,11, true, Encoder.EncodingType.k4X); // Lift DIO, DIO
		encoderArm = new Encoder(12,13, true, Encoder.EncodingType.k4X); // Arm DIO, DIO
		
		encoderArmMap = new EncoderMap(encoderArm); // The Encoder Offset Control
		encoderArmMap.setOffset(5000); // Set the Encoder offset for Mid-Mast
		
		
		controllerDriver = new Joystick(1); // Controller One USB (Logitech Extreme 3D)
		controllerOperator = new Joystick(0); // Controller Two USB (XBOX 360)
		
		lowerLimit = new DigitalInput(17); // DIO
		upperLimit = new DigitalInput(16); // DIO
		flipperUp = new DigitalInput(25); // Mount Sensor
		flipperDown = new DigitalInput(24); // Mount Sensor 
		
		LiftMotor = new VictorSP(1); // PWM
		ArmMotor = new VictorSP(2); // PWM
		IntakeMotor = new VictorSP(14); // PWM
		PivotMotor = new VictorSP(19); // PWM
		
		motorFL = new VictorSP(3); // PWM 
		motorRL = new VictorSP(4); // PWM 
		motorFR = new VictorSP(6); // PWM 
		motorRR = new VictorSP(5); // PWM 
		
		//                     frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor
		MyDrive = new MecDrive(motorFL, motorFR, motorRL, motorRR);
		//MyDrive.setGyro(gyroSensor); // Send the Gyro Object
		MyDrive.invert(true);
		MyDrive.setDeadband(0.12); // The Zone to Ignore
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
		
		SmartDashboard.putNumber("LiftPos", encoderArm.getDistance());
		SmartDashboard.putNumber("LiftArmPos", encoderLift.getDistance());
		
		SmartDashboard.putNumber("EncoderLeft", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("EncoderRight", rightDriveEncoder.getDistance());
		SmartDashboard.putBoolean("UpperLimit", false);
		SmartDashboard.putBoolean("LowerLimits", lowerLimit.get());
		
		//SmartDashboard.putNumber("Battery", pdp.getVoltage());
		//SmartDashboard.putNumber("CurrentAmps", pdp.getTotalCurrent());
		
		// Reset the Encoder Values
		if(SmartDashboard.getBoolean("ResetDriveENC", false) == true){
			SmartDashboard.putBoolean("ResetDriveENC", false);
			leftDriveEncoder.reset();
			rightDriveEncoder.reset();
			
			encoderArm.reset();
			encoderLift.reset();
		}
		
		if(SmartDashboard.getBoolean("ResendAutoCommands", false) == true) {
			SmartDashboard.putData("AutoChoices", chooser); // Resend the AUTO Select
			SmartDashboard.putData("Position", robotPos); // Resend the POSITION Select
			
			SmartDashboard.putBoolean("ResendAutoCommands", false); // Rest Checkbox
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
		MyDrive.stop(); // Send Stop Message to all drive motors during Stop Command.
		//Scheduler.getInstance().run();
	}
	
	@Override
	public void testPeriodic(){
		liftBreak.set(Value.kReverse); // Set the Lift Break to be On
		clawPinch.set(Value.kForward); // Set the Pinch to be Close
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
	/**
	 * This Section of the Code (autonomousInit) is just going to be doing some
	 *  large sections of if and self with some switch cases to make the auto
	 *  select the correct program to run depending on the ENV made on the field.
	 */
	/**
	 * TODO: Fix Auto to work with the new Controls for the Lift and Pivot Arm. 
	 */
	@Override
	public void autonomousInit() {
		int baseLine = (128*EncoderIN); // Set the Total Distance in the Encoder's Ticks by EncoderIN * the target distance. 
		// Set the init State, Set asap.
		
		timer.reset();
		timer.start();
		CommandGroup autoRuntime = new CommandGroup();
		
		// This is a FIX for when the Driver Station does not load and the Auto Command Data is not set.
		try {
			
			compressor0.setClosedLoopControl(true);
			autonomousCommand = this.chooser.getSelected();
			SmartDashboard.putString("AutoSelect", autonomousCommand);
			String autoSelect = autonomousCommand;
			String robotPos = this.robotPos.getSelected();
			
			/**
			 * TODO: REVIEW CODE, VERIFY DEFINITION NAMES ARE MATCHING FUNCTION AND DEVICE LOCATION
			 */
			// Setup the Static Systems and their Dependent Resources.
			new DriveCTRL(MyDrive, leftDriveEncoder, rightDriveEncoder);
			new TurnCTRL(MyDrive, gyroSensor);
			new DistanceAlign(MyDrive, distSensor);
			
			new ArmCTRL(ArmMotor, encoderLift, upperLimit, lowerLimit); // This is just the Mec that holds the cube
			new ArmTimed(ArmMotor, upperLimit, lowerLimit); // This is just the Mec that holds the cube
			new ArmCTRLHigh(ArmMotor, upperLimit, lowerLimit); // This is just the Mec that holds the cube
			new LiftCTRL(LiftMotor, encoderArmMap, liftBreakMap); // This is the Entire Lift
			
			/**
			 * DriverStation.getInstance().getGameSpecificMessage() returns the Data of the Left or Right 
			 *      Direction of your Color on the Switch Scale Switch.
			 * The First Char of the String will ALWAYS be the once closest to your drive station wall.
			 * 
			 * if( GamePos[0] == 'L' ) // The Close Switch Left Side is your Color.
			 * if( GamePos[1] == 'L' ) // Scale, Center Field, Left Side
			 * if( GamePos[2] == 'L' ) // The Far Switch Left Side is your Color.
			 */
			this.GamePos = DriverStation.getInstance().getGameSpecificMessage().toUpperCase().toCharArray();
	
			// Add the Timer Delay Action/Command
			autoRuntime.addSequential( new WaitCTRL( (int) SmartDashboard.getNumber("AutoDelay", 0)) );
			
			///////////////////////////////////////////////////////////////////
			/////////////  STAGE 1 ////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////
	
			//   +-+-+-+ +-+-+-+-+
			//   |P|R|E| |E|X|E|C|
			//   +-+-+-+ +-+-+-+-+
			// This is used to find out if the scale is our color
			//  then to see if the switch is out color then RESET the auto Accordingly.
			if( autonomousCommand == ScaleSwitchAuto ){
				System.out.println("Auto Decisioning Auto...");
				
				// Left Side
				if(robotPos == ROBOT_POS_ONE_STR){
					// Scale Detection
					if(GamePos[1] == 'L'){
						autonomousCommand = ScaleAuto;
					}
					
					// Switch Detection
					else if( GamePos[0] == 'L' ){
						autonomousCommand = SwitchAuto;
					}
				}
				
				else if (robotPos == ROBOT_POS_TWO_STR) {
					// Do nothing.
					autonomousCommand = NoAuto;
					log("(CENTER) Robot on the Left, Robot on the Right, Stuck in the Middle with you!");
				}
				
				// Right Side
				else if (robotPos == ROBOT_POS_THREE_STR) {
					// Scale Detection
					if(GamePos[1] == 'R'){
						autonomousCommand = ScaleAuto;
					}
					
					// Switch Detection
					else if( GamePos[0] == 'R' ){
						autonomousCommand = SwitchAuto;
					}
				}
				
				log("Auto Select: " + autonomousCommand);
			} // End "ScaleSwitchAuto" Selection Process.
			
			///////////////////////////////////////////////////////////////////
			/////////////  STAGE 2 ////////////////////////////////////////////
			///////////////////////////////////////////////////////////////////
			
			//   +-+-+ +-+-+-+-+
			//   |N|O| |A|U|T|O|
			//   +-+-+ +-+-+-+-+
			if(autonomousCommand == NoAuto){
				System.out.println("HUMAN: :'( Told not to do an Auto. [NoAuto]");
				leftDriveEncoder.reset();
				rightDriveEncoder.reset();
			} // End "NoAuto" Selection Process.
			
	
			//   +-+-+-+-+-+
			//   |S|C|A|L|E|
			//   +-+-+-+-+-+
			// Auto for the Scale
			// TODO: Make SCALE auto.
			else if(autoSelect == ScaleAuto){
				// If you are in POS One or POS Two
				if(robotPos == ROBOT_POS_ONE_STR || robotPos == ROBOT_POS_THREE_STR){
					
					if(GamePos[1] == 'L') {
						log("POS 1 - Left Color");
						
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get past the Base line
						autoRuntime.addSequential( new WaitCTRL(0.5) ); // Wait one Sec
						
						// Does not work for an auto align since the Distance sensor does not work.
						//autoRuntime.addSequential( new DistanceAlign(0.6, 34) ); // Measure from the Switch, 24in from the Switch past the baseline.
	
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get to the Scale.
						autoRuntime.addSequential( new TurnCTRL(0.5, 85) ); // Turn to the Right
						
						solenoidUpdate(false, liftBreakOn, liftBreakOff); // Break Off
						autoRuntime.addSequential( new ArmCTRLHigh(0.8) ); // Lift arm upto the Limit
						solenoidUpdate(true, liftBreakOn, liftBreakOff); // Break On
						
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 50) ); // Drive Forward to the Switch
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						//autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
						autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, -50) ); // Drive Backwards from the Switch
						
					}
					else if(GamePos[1] == 'R') {
						log("POS 1 - Right Color");
						
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get past the Base line
						autoRuntime.addSequential( new WaitCTRL(0.5) ); // Wait one Sec
						
						// Does not work for an auto align since the Distance sensor does not work.
						//autoRuntime.addSequential( new DistanceAlign(0.6, 34) ); // Measure from the Switch, 24in from the Switch past the baseline.
	
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get to the Scale.
						autoRuntime.addSequential( new TurnCTRL(0.5, -85) ); // Turn to the Left
						
						solenoidUpdate(false, liftBreakOn, liftBreakOff); // Break Off
						autoRuntime.addSequential( new ArmCTRLHigh(0.8) ); // Lift arm upto the Limit
						solenoidUpdate(true, liftBreakOn, liftBreakOff); // Break On
						
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 50) ); // Drive Forward to the Switch
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						//autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
						autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, -50) ); // Drive Backwards from the Switch
					}
					
				}
				else if(robotPos == ROBOT_POS_TWO_STR){
					log("POS 2 - ERROR");
					System.out.println("HUMAN: Not in Left or Right Slot, Can't Auto!. [Scale>Center]");
				}
				
				
				
			} // End "ScaleAuto" Selection Process.
			
	
			//   +-+-+-+-+-+-+
			//   |S|W|I|T|C|H|
			//   +-+-+-+-+-+-+
			// Switch Auto
			else if(autoSelect == SwitchAuto){
				
				//   +-+-+-+-+
				//   |L|E|F|T|
				//   +-+-+-+-+
				
				// If our Color is on the LEFT side and we are in POS 1 (Right Side of the Field)
				// Be sure to Check the Color of the Scale or Switch before running and the Placement.
				if(GamePos[0] == 'L' && robotPos == ROBOT_POS_ONE_STR){
					
					log("POS 1 - Left Color");
					
					autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get past the Base line
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new ArmCTRL(0.8, 380) ); // Lift arm up to the target 100
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new TurnCTRL(0.5, 85) ); // Turn to the Right
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 100) ); // Drive Forward to the Switch
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					//autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
					autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, -50) ); // Drive Backwards from the Switch
				}
				
		
				//   +-+-+-+-+-+
				//   |R|I|G|H|T|
				//   +-+-+-+-+-+
	
				// If our Color is on the RIGHT side and we are in POS 3 (Right Side of the Field)
				// Be sure to Check the Color of the Scale or Switch before running and the Placement.
				else if(GamePos[0] == 'R' && robotPos == ROBOT_POS_THREE_STR){
					
					log("POS 3 - Right Color");
					
					autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get past the Base line
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new ArmCTRL(0.8, 380) ); // Lift arm up to the target 100
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new TurnCTRL(0.5, -85) ); // Turn to the Right
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 100) ); // Drive Forward to the Switch
					autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
					//autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
					autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, -50) ); // Drive Backwards from the Switch
				}
				
				// CENTER AUTO, Dual Auto Switch Case
				// TODO: Create Center Auto Options and the Controls
				else if(robotPos == ROBOT_POS_TWO_STR){
					
					if(GamePos[0] == 'L'){
						// If our Color in on the left side.
						
						log("POS 2 - Left Color");
						
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 20*EncoderIN) ); // Crab Left, Color is on the Left
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new ArmCTRL(0.8, 380) ); // Lift arm up to the target 100
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0, 0, -0.8, 58*EncoderIN) ); // Drive Forward to the Switch
						////autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, 62*EncoderIN) ); // Drive Backwards from the Switch
						autoRuntime.addSequential( new MotorTimed(IntakeMotor, -0.9, 2) ); // Spit out the Cube from the Claw
					}
					else if(GamePos[0] == 'R') {
						// If out Color is on the right side.
						
						log("POS 2 - Right Color");
						
						// FLIPED CONTORLS
						autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, 20*EncoderIN) ); // Crab Left, Color is on the Left
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new ArmCTRL(0.8, 380) ); // Lift arm up to the target 100
						autoRuntime.addSequential( new WaitCTRL(1.0) ); // Wait one Sec
						autoRuntime.addSequential( new DriveCTRL(0, 0, 0.8, 48*EncoderIN) ); // Drive Forward to the Switch
						////autoRuntime.addSequential( new SolenoidCTRL(ClawMap, false) ); // Change State
						autoRuntime.addSequential( new DriveCTRL(-0.5, 0, 0, 62*EncoderIN) ); // Drive Backwards from the Switch
						autoRuntime.addSequential( new MotorTimed(IntakeMotor, -0.9, 2) ); // Spit out the Cube from the Claw
					}
					
				}
				
				else {
					// If no Case Matches, Figure out if you can drive the baseline.
					
					if(robotPos == ROBOT_POS_ONE_STR || robotPos == ROBOT_POS_THREE_STR){
						// Run baseline auto to get Points
						autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) ); // Get past the Base line
					}
					else {
						log("Can not even do a Baseline auto.");
					}
					
					
				}
				
			} // End "Switch" Selection Process.
			
	
			//   +-+-+-+-+-+-+-+-+
			//   |B|A|S|E|L|I|N|E|
			//   +-+-+-+-+-+-+-+-+
			// Drive Past the Base Line
			else if(autoSelect == BaseLine){
				
				// Drive past the Baseline if not Center
				if(robotPos == ROBOT_POS_ONE_STR || robotPos == ROBOT_POS_THREE_STR){
					log("POS 1/3 - Unknown Side, Baseline");
					autoRuntime.addSequential( new DriveCTRL(0.5, 0, 0, baseLine) );
				}
				
				else if ( robotPos == ROBOT_POS_TWO_STR){
					log("POS 2 - ERRROR");
					System.out.println("HUMAN: Not in Left or Right Slot, Can't Auto!. [Baseline>Center]");
				}
				
				
			} // End "Baseline" Selection Process.
		
			else {
				log("Unknown Auto Routine for auto \"" + autoSelect + "\", POS: " + robotPos );
			}
			
			SmartDashboard.putString("AutoSelect", autonomousCommand);
		}
		catch ( NullPointerException e) {
			System.out.println("AUTO:  NO POINTER");
		}
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Schedule the autonomous command
		if (autoRuntime != null)
			autoRuntime.start();
		SmartDashboard.putBoolean("AutoInitComplete", true);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * Set Init Values and Settings for Teleop
	 */
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
		// Driver B Button (Logitech 3D) [MyDrive]
		// This Sets all of the Motors to a Stop State when Pressed.
			double powerJoy = -(controllerDriver.getRawAxis(1));
			double turnJoy = controllerDriver.getRawAxis(2);
			double crabJoy = controllerDriver.getRawAxis(0);
			
		// A button on XBOX and Thumb Button on Driver 3D Controller
			if(controllerDriver.getRawButton(2) || controllerOperator.getRawButton(2) ){
				SmartDashboard.putBoolean("UserStop", true);
				MyDrive.DriveControl.stopMotor();
			}
			else {
				SmartDashboard.putBoolean("UserStop", true);
				// This will scale all of the values down.
				if(!controllerDriver.getRawButton(1)) {
					powerJoy = powerJoy * 0.6; // Make the Power half speed when not pushed.
					turnJoy = turnJoy * 0.6;
				}
				
				if(between(crabJoy, 0.2)) {
					crabJoy = 0;
				}
				
				
				MyDrive.drive(powerJoy, turnJoy, crabJoy);
			}
			
			SmartDashboard.putNumber("DrivePower", powerJoy ); // Send Value to the Dashboard
			SmartDashboard.putBoolean("LimitedPower", controllerDriver.getRawButton(1) );
			SmartDashboard.putNumber("DriveTurn", turnJoy ); // Send Value to the Dashboard
			SmartDashboard.putNumber("DriveCrab", crabJoy ); // Send Value to the Dashboard
		
		// Take in Cube and Push out Cube [IntakeMotor]
		// Operator A Button (XBOX) and Trigger on Driver Controller (Logitech 3D)
			if( controllerOperator.getRawButton(4) ){
				// Put out Cube
				controllerOperator.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4 );
				SmartDashboard.putString("CubeIntake", "Out");
				IntakeMotor.set(-0.9);
			}
			else if( controllerOperator.getRawButton(1) ){
				// Take in Cube
				controllerOperator.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4 );
				SmartDashboard.putString("CubeIntake", "In");
				IntakeMotor.set(0.9);
			}
			else {
				// No State, Normal
				SmartDashboard.putString("CubeIntake", "Idle");
				controllerOperator.setRumble(GenericHID.RumbleType.kLeftRumble, 0 );
				IntakeMotor.set(0);
			}
		
		// Operator 
		// Motor Arm Lift [liftBreakMap , LiftMotor]
			double lValue = -(controllerOperator.getRawAxis(5));
			// Negative Number is Up for the Motor Output
			
			if( lValue > 0.2){
				// Up
				if(encoderArm.getDistance() <= MaxHightEncoder || upperLimit.get() == true ) {
					// Past Max Height
					System.out.println("L: Up -- Max Height Reached.");
					liftBreak.set(Value.kReverse); // Break On
					if(!controllerOperator.getRawButton(5)) {
						LiftMotor.setSpeed(0);
					}
					else {
						LiftMotor.setSpeed( lValue );	
					}
				}
				else {
					// UP
					System.out.println("L: Up");
					liftBreak.set(Value.kForward); // Break Off
					LiftMotor.setSpeed( lValue );
				}
			}
			else if(lValue < -0.2) {
				// DOWN
				// Keep in mind the Lower Limit Switch is Always True till the Arm Comes down and interrupts the Light, Makes it False.
				if(lowerLimit.get()){
					// DOWN
					System.out.println("L: Down");
					// Light is on by Default (Reflecting and getting Light Normal, When the Arm is not down.)
					liftBreak.set(Value.kForward); // Break Off
					LiftMotor.setSpeed( lValue );
				}
				else {
					// Auto Limit and Reset for Calibration of the Lift Sensor
					// Reached Lower Limit
					System.out.println("L: Down -- Lower Limit Reached.");
					encoderArm.reset();  // Zero Sonar
					liftBreak.set(Value.kReverse); // Break On
					LiftMotor.setSpeed(0);
				}
			}
			else {
				// OFF, IDLE
				
				// User Manual Break Override, Switched from Button 5 to Button 6
				if(controllerOperator.getRawButton(6)) {
					liftBreak.set(Value.kForward); // Break Off
				}
				else {
					liftBreak.set(Value.kReverse); // Break On
				}
				
				LiftMotor.setSpeed(0);
			}

		// Operator
		// This is for the Arm Lift Control [armBreakMap, ArmMotor]
			if( !between(controllerOperator.getRawAxis(1), 0.2) ) {
				double CtrlArmValue = controllerOperator.getRawAxis(1)*0.75; // Scale Value input to only 60% (0-0.6)
				System.out.println("L: "+CtrlArmValue);
				CtrlArmValue = -(CtrlArmValue); // Fix Direction of the Lift Motor Output to go the correct direction
				ArmMotor.setSpeed(CtrlArmValue);
				armBreak.set(Value.kReverse); // Break Off
			}
			else {
				// OFF, IDLE
				
				// User Manual Break Override, Switched from Button 6 to Button 5
				if(controllerOperator.getRawButton(5)) {
					armBreak.set(Value.kReverse); // Break Off
				}
				else {
					armBreak.set(Value.kForward); // Break On
				}
				ArmMotor.setSpeed(0.0);
			}
			
		// Open Claw when the Trigger is pushed on the Operator Controller
			if( controllerOperator.getRawAxis(3) > 0.3 ) {
				clawPinch.set(Value.kReverse); // Set the Pinch to be Open
			}
			else {
				clawPinch.set(Value.kForward); // Set the Pinch to be Close
			}
			
		Scheduler.getInstance().run();
	}
	
	
	/**
	 * 
	 * @param Input
	 * @param StartRange
	 * @param EndRange
	 * @return Boolean Value that tells if it is in between the range StartRange and EndRange.
	 */
	public boolean inRange(double Input, double StartRange, double EndRange) {
		if(Input < StartRange && Input > EndRange) {
	        return true;
	    }
		return false;
	}
	
	/**
	 * 
	 * @param InValue
	 * @param BetweenValue
	 * @return Boolean Value that tells if it is in between the range from InValue and 0
	 */
	public boolean between(double InValue, double BetweenValue ) {
		InValue = Math.abs(InValue);
		BetweenValue = Math.abs(BetweenValue);
		
		if(InValue > BetweenValue) {
			return false;
		}
		return true;
	}
	
	/**
	 * Return the Gyro or VGyro Value
	 * @return Boolean Value
	 */
	public double getGyro(){
		return this.gyroSensor.getAngle();
	}
	public double getVGyro(){
		return ( this.gyroSensor.getAngle() + this.VirtualOffset);
	}
	
	/**
	 * Zero the Gyro Controls
	 * Zero the Virtual Gyro Controls
	 */
	public void zeroGyro(){
		this.gyroSensor.reset();
	}
	public void zeroVGyro(){
		this.VirtualOffset = -getGyro();
	}
	
	/**
	 * Returns the Max Distance of the robot based on the left or right encoder.
	 * @return Int Encoder Value
	 */
	public int getDriveEncoder(){
		return (int) Math.max(
			Robot.leftDriveEncoder.getDistance(),
			Robot.rightDriveEncoder.getDistance()
		);
	}
	
	public void resetEncoder(){
		Robot.leftDriveEncoder.reset();
		Robot.rightDriveEncoder.reset();
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
	 * Used to set the State for the Solenoid Controls
	 * @param NewState NewState The State to set on the solenoid
	 * @param Port1 Solenoid When State is True it is Set to True
	 * @param Port2 Port2 When State is True it is Set to False 
	 */
	public void solenoidUpdate(boolean NewState, Solenoid Port1, Solenoid Port2){
		Port1.set(NewState);
		Port2.set(!NewState);
	}
	
	/**
	 * 
	 * @param Log Text to Log
	 */
	public void log(String Log){
		System.out.println(Log);
	}
	
}
