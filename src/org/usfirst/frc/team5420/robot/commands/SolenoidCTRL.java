package org.usfirst.frc.team5420.robot.commands;

import org.usfirst.frc.team5420.robot.SolenoidMap;

import edu.wpi.first.wpilibj.command.Command;

public class SolenoidCTRL extends Command {

	boolean isDone = false;
	SolenoidMap SOLENOID = null;
	boolean newState = false;
	
	/**
	 * Action INIT
	 * 
	 * @param Speed      Speed used to drive the motor
	 * @param targetArm  The Target Encoder Value
	 */
	public SolenoidCTRL (SolenoidMap CTRL, boolean Open){
		this.SOLENOID = CTRL;
		this.newState = Open;
	}	
	
	@Override
	public void initialize(){
		
	}
	
	@Override
	protected void execute() {
		this.SOLENOID.set(this.newState);
		this.isDone = true;
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}