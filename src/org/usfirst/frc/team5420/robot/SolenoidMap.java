package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidMap {

	public Solenoid SolenoidOne = null;
	public Solenoid SolenoidTwo = null;
	public boolean newState = false;
	
	/**
	 * This is the INIT to have a Pair of Solenoids Operate Together
	 * 
	 * @param One  Solenoid A
	 * @param Two  Solenoid B
	 */
	public SolenoidMap ( Solenoid One, Solenoid Two ) {
		this.SolenoidOne = One;
		this.SolenoidTwo = Two;
		this.newState = One.get(); // Set the Instance's Current State 
	}
	
	/**
	 * This is the INIT to have a Pair of Solenoids Operate Together
	 * 
	 * @param One    Solenoid A
	 * @param Two    Solenoid B
	 * @parma State  Defines the Starting State
	 */
	public SolenoidMap ( Solenoid One, Solenoid Two, boolean State ) {
		this(One, Two); // Call the Parent INIT to setup the config.
		this.newState = State; // Overwrite the Stored State with our own.
	}
	
	public boolean get(){
		return this.newState;
	}
	
	public void set(boolean newStateIn){
		this.newState = newStateIn;
		this.update();
	}
	
	/**
	 * Sets the State for SolenoidOne to be ON and SolenoidTwo to be OFF
	 */
	public void open(){
		this.newState  = true;
		this.update();
	}
	
	/**
	 * Set the Sate for SolenoidOne to be OFF and SolenoidTwo to be OFF
	 */
	public void close(){
		this.newState = false;
		this.update();
	}
	
	/**
	 * Switches the Current State in the Solenoid
	 */
	public void invert(){
		this.newState = !this.newState;
		this.update();
	}
	
	/**
	 * Set the State to the last stored states in the instance.
	 * NOTE: Does not need to be called seperatly to change states!
	 */
	public void update(){
		this.SolenoidOne.set(newState);
		this.SolenoidTwo.set(!newState);
	}
		
}
