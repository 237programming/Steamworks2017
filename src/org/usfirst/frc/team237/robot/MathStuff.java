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
			return (angle - 0)*(1023-512)/(180 - 0) + 512;
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
	public static boolean isInRange(double val, double upperBound, double lowerBound )
	{
		boolean flipped = false;
		if (upperBound < lowerBound )
		{
			double temp = upperBound;
			upperBound = lowerBound;
			lowerBound = temp;
			flipped = true; 
		}
		if (val <= upperBound && val >= lowerBound )
		{
			return !flipped; 
		} 
		else 
		{
			return flipped; 
		}
	}
	public static double normalizeEncInput(double enc)
	{
		if (enc > 1023)
		{
			enc -= 1024; 
		} else if (enc < 0) {
			enc += 1024;
		}
		return enc; 
	}
}