package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidMap {

	public Solenoid SolenoidOne = null;
	public Solenoid SolenoidTwo = null;
	public boolean newState = false;
	
	/**
	 * This is the INIT to have a Pair of Solenoids Operate Together.
	 * This class is a lot like the DoubleSolenoid Class but it allows
	 *   direct calls and creations of objects by creating pairs.
	 *   
	 * @see edu.wpi.first.wpilibj.DoubleSolenoid
	 * @param One  Solenoid A
	 * @param Two  Solenoid B (INVERTED CONTROL)
	 */
	public SolenoidMap ( Solenoid One, Solenoid Two ) {
		this.SolenoidOne = One;
		this.SolenoidTwo = Two;
	}
	
	/**
	 * This is the INIT to have a Pair of Solenoids Operate Together
	 * 
	 * @param One  Solenoid A
	 * @param Two  Solenoid B (INVERTED CONTROL)
	 */
	public SolenoidMap ( int One, int Two ) {
		Solenoid OneIn = new Solenoid(One);
		Solenoid TwoIn = new Solenoid(Two);
		
		this.SolenoidOne = OneIn;
		this.SolenoidTwo = TwoIn;
	}
	
	/**
	 * Sets the Input to the selected boolean value and the second Solenoid opposite.
	 * @param newStateIn The new State to set.
	 */
	public void set(boolean newStateIn){
		this.SolenoidOne.set(newStateIn);
		this.SolenoidTwo.set(!newStateIn);
	}
	
	/**
	 * Sets the Input to the selected boolean value and the second Solenoid opposite.
	 * @param newStateIn The new State to set.
	 */
	public void set(Value NewState) {
		if(NewState == Value.kForward) {
			this.open();
		}
		else if(NewState == Value.kReverse) {
			this.close();
		}
		else if(NewState == Value.kOff){
			this.off();
		}
		
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
	
	
	/**
	 * Gets the First controller (as it is the normal state) and return it.
	 * @return Bollean The current State of the controller.
	 */
	public boolean get(){
		return this.SolenoidOne.get();
	}
		
}
