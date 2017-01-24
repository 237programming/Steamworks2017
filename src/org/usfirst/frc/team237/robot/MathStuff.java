package org.usfirst.frc.team237.robot;

public final class MathStuff {
	public static double encoderToDegrees(double encoderCount)
	{
		return (encoderCount/1023f * 359f);
	}
	public static int degreeToEncoder(double degree)
	{
		double res = (degree/359 * 1023);
		return (int) Math.round(res);
	}
	public static double mapAngleToEnc(double angle)
	{
		return (angle - -180)*(1023-0)/(180 - -180) + 0;
	}
	public static double mapEncToAngle(double enc)
	{
		return (enc - 0)*(180- -180)/(1023 - 0) + -180;
	}
}