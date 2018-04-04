package org.usfirst.frc.team5420.util;

/**
 * Class that represents data packet being received from Pixy
 * 
 * Used from team 1089 (Mercury)
 * @link https://github.com/Mercury1089/2018-robot-code/blob/64d25c6ad5024f6bb7dce81629c29a2601f39163/robot/src/org/usfirst/frc/team1089/util/BoundingBox.java
 */
public class BoundingBox implements Comparable<BoundingBox>{
	private final int X;
	private final int Y;
	private final int WIDTH;
	private final int HEIGHT;

	public BoundingBox(int nX, int nY, int nW, int nH) {
		X = nX;
		Y = nY;
		WIDTH = nW;
		HEIGHT = nH;
	}

	public int getArea() {
		return WIDTH * HEIGHT;
	}

	public int getX() {
		return X;
	}

	public int getY() {
		return Y;
	}

	public int getWidth() {
		return WIDTH;
	}

	public int getHeight() {
		return HEIGHT;
	}

	@Override
	public int compareTo(BoundingBox boundingBox) {
		int areaA = getArea(), areaB = boundingBox.getArea();
		return (int)Math.signum(areaA - areaB);
	}
}