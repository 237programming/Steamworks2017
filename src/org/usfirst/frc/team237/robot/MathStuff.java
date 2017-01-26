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
		if (angle>=0){
			return (angle - 0)*(1023-512)/(180 - 0) + 0;
		} 
		else {
			return (angle - -180)*(511 - 0)/(-1 - -180) + 0;
		}
		
	}
	public static double mapEncToAngle(double enc)
	{
		if (enc>=512){
			return (enc - 512)*(180-0)/(1023 - 512) + 0;
		} 
		else {
			return (enc - 0)*(0 - -180)/(511 - 0) + -180;
		}
	}
	public static double mapStickToAngle(double stick)
	{
		return (stick - -1)*(180 - -180)/(1 - -1) + -180;
	}
}