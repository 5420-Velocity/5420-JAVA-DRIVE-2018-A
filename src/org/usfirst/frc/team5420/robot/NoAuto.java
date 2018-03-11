package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.command.Command;

public class NoAuto extends Command {
	
	private boolean isFinished = false;
	public MecDrive DriveControl = null;
	private int step;
	
	public NoAuto (){
		
	}
	
	// Called just before this Command runs the first time
	@Override
	protected void initialize(){
		this.isFinished = false;
	}
	
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// The Switch with the Step int, you are able to move 
		//   to different types of actions dependent on actions.
		switch (this.step){
			
			case 0:
				// Used for Pre-Flight Checks
				next_step();
			break;
			
			
			case 999:
			default:
				// If no more cases are left and they are not caught, Finish the Command.
				this.isFinished = true;
			break;
		}
		this.isFinished = true;
	}
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	public void next_step(){
		this.step++;
	}
	public void jumpto(int j){
		this.step = j;
	}
	public void end_steps(){
		this.step = 999;
	}
	
}
