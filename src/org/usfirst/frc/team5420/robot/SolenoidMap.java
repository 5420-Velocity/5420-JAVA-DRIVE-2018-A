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
	 * @param Two  Solenoid B (INVERTED CONTROL)
	 */
	public SolenoidMap ( Solenoid One, Solenoid Two ) {
		this.SolenoidOne = One;
		this.SolenoidTwo = Two;
	}
	
	public void set(boolean newStateIn){
		this.SolenoidOne.set(newStateIn);
		this.SolenoidTwo.set(!newStateIn);
	}
	
	/**
	 * Sets the State for SolenoidOne to be ON and SolenoidTwo to be OFF
	 * In DoubleSolenoid it would be Value.kForward
	 */
	public void open(){
		this.SolenoidOne.set(true);
		this.SolenoidTwo.set(false);
	}
	
	/**
	 * Set the Sate for SolenoidOne to be OFF and SolenoidTwo to be ON
	 * In DoubleSolenoid it would be Value.kReverse
	 */
	public void close(){
		this.SolenoidOne.set(false);
		this.SolenoidTwo.set(true);
	}
	

	/**
	 * Set the Sate for SolenoidOne to be OFF and SolenoidTwo to be OFF
	 * In DoubleSolenoid it would be Value.kOff
	 */
	public void off(){
		this.SolenoidOne.set(false);
		this.SolenoidTwo.set(false);
	}
		
}
