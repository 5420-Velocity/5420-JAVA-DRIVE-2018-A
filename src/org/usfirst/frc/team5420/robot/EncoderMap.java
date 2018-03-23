package org.usfirst.frc.team5420.robot;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderMap {
	
	private Encoder source = null;
	private int offset = 0;
	
	/**
	 * This will init and store the Encoder plus an Offset
	 * @param input
	 * @param offset
	 */
	public EncoderMap(Encoder input, int offset) {
		this.source = input;
		this.offset = offset;
	}
	
	/**
	 * This will init and store the Encoder
	 * @param input
	 */
	public EncoderMap(Encoder input) {
		this.source = input;
	}
	
	/**
	 * Returns the Offset value stored
	 * @return Stored Offset
	 */
	public int getOffset(){
		return this.offset;
	}
	
	/**
	 * This will set the Distance
	 * @param offset
	 */
	public void setOffset(int offset) {
		this.offset = offset;
	}
	
	/**
	 * This will take the value you input and add it to the Stored Input
	 * @param offAdd
	 */
	public void addOffset(int offAdd){
		this.offset += offAdd;
	}
	
	/**
	 * Return the Distance with the offset
	 * @return Retuns the Value + the Offset
	 */
	public double getDistance() {
		return source.getDistance()+this.offset; 
	}
	
}
