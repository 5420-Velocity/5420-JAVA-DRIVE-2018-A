package org.usfirst.frc.team5420.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team5420.robot.MecDrive;

import edu.wpi.first.wpilibj.Ultrasonic;

public class DistanceAlign extends Command {

	private MecDrive DriveSystem = null;
	private Ultrasonic sensorDevice= null;
	private int Distance = 0;
	private double Speed = 0;
	private boolean overDistance = false;
	private boolean isDone = false;
	
	public DistanceAlign(MecDrive DriveInput, Ultrasonic sensor){
		this.DriveSystem = DriveInput;
		this.sensorDevice = sensor;
	}
	
	public DistanceAlign(double Speed, int distance) {
		this.Distance = distance; // The total distance to mesure from the WALL or SWITCH
		this.Speed = Math.abs(Speed);
	}

	@Override
	public void initialize(){
		if(this.sensorDevice.getRangeInches() >= this.Distance){
			// Greater Distance
			this.overDistance = true;
			this.Speed = - Math.abs( this.sensorDevice.getRangeInches() );
		}
		else if (this.sensorDevice.getRangeInches() <= this.Distance) {
			// Lower Distance
			this.overDistance = false;
			this.Speed = Math.abs( this.sensorDevice.getRangeInches() );
		}
	}
	
	@Override
	public void execute(){
		
		// OVER UNDER DISTANCE
		if(this.overDistance == true){
			// Past the Distance, Do checks as driving
			if(this.sensorDevice.getRangeInches() >= this.Distance){
				// Greater Distance, Driving
				this.DriveSystem.drive(0, 0, this.Speed);
			}
			else {
				this.DriveSystem.drive(0, 0, 0);
				this.isDone = true;
			}
		}
		
		// OVER INIT DISTANCE
		else if(this.overDistance == false){
			// Not Past the Distance, Do checks as driving
			if (this.sensorDevice.getRangeInches() <= this.Distance) {
				// Lower Distance, Driving
				this.DriveSystem.drive(0, 0, this.Speed);
			}
			else {
				this.DriveSystem.drive(0, 0, 0);
				this.isDone = true;
			}
		}
			
	}
	
	@Override
	protected boolean isFinished() {
		return this.isDone;
	}

}
