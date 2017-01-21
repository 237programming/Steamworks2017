package org.usfirst.frc.team237.robot;

public final class MathStuff {
	public static int encoderToDegrees(int encoderCount)
	{
		return (int) (encoderCount/1023f * 360f);
	}
}