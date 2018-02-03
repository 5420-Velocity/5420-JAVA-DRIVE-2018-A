/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public DriverStation.Alliance color;
	public double time;
	public static OI jio;
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	// Setup of the Devices in the Code.
	Timer timer = new Timer();
	public ADXRS450_Gyro gyroSensor;
	
	public static Compressor compressor0;
	public static  Solenoid solenoid0, solenoid1;
	
	public static Encoder encoder0, encoder1;
	public static DigitalInput UpperLimit, LowerLimit;
	
	public static Joystick joystick0, joystick1;
	public static VictorSP LiftMotor;
	
	// Motor Setup
	public VictorSP motorFL, motorBL, motorBR, motorFR;
	public MecDrive MyDrive;
	
	public char[] GamePos;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putBoolean("AutoInitComplete", false);
		SmartDashboard.putBoolean("TeleopInitComplete", false);
		SmartDashboard.putBoolean("RobotInit", false);
		
		color = DriverStation.getInstance().getAlliance();
		time = DriverStation.getInstance().getMatchTime();
		
		jio = new OI();
		
		gyroSensor = new ADXRS450_Gyro( SPI.Port.kOnboardCS0 ); // SPI
		gyroSensor.calibrate(); // SPI
		
		compressor0 = new Compressor(0);
		solenoid0 = new Solenoid(1); // Claw Close 
		solenoid1 = new Solenoid(2); // Claw Open
		
		encoder0 = new Encoder(4,5, false, Encoder.EncodingType.k4X); // Left DIO, DIO
		encoder1 = new Encoder(6,7, false, Encoder.EncodingType.k4X); // Right DIO, DIO
		
		joystick0 = new Joystick(0); //Controller One USB
		joystick1 = new Joystick(1); //Controller Two USB
		
		UpperLimit = new DigitalInput(0); // DIO
		LowerLimit = new DigitalInput(9); // DIO
		
		LiftMotor= new VictorSP(2); // PWM
		
		motorFL= new VictorSP(3); // PWM
		motorBL= new VictorSP(4); // PWM
		
		motorBR= new VictorSP(5); // PWM
		motorFR= new VictorSP(6); // PWM
		
		MyDrive = new MecDrive(motorFL, motorBL, motorBR, motorFR, 0.1);
		
		SmartDashboard.putBoolean("RobotInit", true);
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
		
		heartbeat();
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
		timer.reset();
		timer.start();
		compressor0.setClosedLoopControl(true);
		autonomousCommand = chooser.getSelected();
		this.GamePos = DriverStation.getInstance().getGameSpecificMessage().toCharArray();
		/**
		 * DriverStation.getInstance().getGameSpecificMessage() returns the Data of the Left or Right 
		 *      Direction of your Color on the Switch Scale Switch.
		 * The First Char of the String will ALWAYS be the once closest to your drive station wall.
		 * 
		 * if( gameData.charAt(0) == 'L' ) // The Close Switch's Left Side is your Color.
		 * if( gameData.charAt(1) == 'L' ) // Scale's Left Side is your Color.
		 * if( gameData.charAt(2) == 'L' ) // The Far Switch's Left Side is your Color.
		 */
		
		
		String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
			case "My Auto":
				
			break;
			case "Default Auto":
			default:
				//autonomousCommand = new ExampleCommand();
			break;
		}
		 

		// Schedule the autonomous command
		if (autonomousCommand != null)
			autonomousCommand.start();
		SmartDashboard.putBoolean("AutoInitComplete", true);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		heartbeat();
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		SmartDashboard.putBoolean("TeleopInitComplete", true);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		double powerJoy = joystick0.getRawAxis(0);
		double turnJoy = joystick0.getRawAxis(1);
		double crabJoy = joystick0.getRawAxis(2);
		MyDrive.drive(powerJoy, turnJoy, crabJoy);
		
		SmartDashboard.putNumber("DrivePower", powerJoy );
		SmartDashboard.putNumber("DriveTurn", turnJoy );
		SmartDashboard.putNumber("DriveCrab", crabJoy );
		
		heartbeat();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}
	
	
	/**
	 * This Smartdashboard 'RunningHeartbeat' is updating the Runtime value of the Robot.
	 * Used in the dashboard to see if the robot is running. Send the Unix Time of the RIO.
	 * @link https://stackoverflow.com/a/732043/5779200
	 */
	public void heartbeat(){
		time = DriverStation.getInstance().getMatchTime();
		SmartDashboard.putNumber("MatchTime", time);
		SmartDashboard.putNumber("SystemTime", System.currentTimeMillis() / 1000L);
		SmartDashboard.putNumber("RunningHeartbeat", timer.get());
	}
}
