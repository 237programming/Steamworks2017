package org.usfirst.frc.team237.robot;

public final class MathStuff {
	
	//Convert encoder counts (0 to 1023) to degrees (0 to 359);
	public static double encoderToDegrees(double encoderCount)
	{
		return (encoderCount/1023f * 359f);
	}
	
	//Convert degrees (0 to 359) to encoder counts (0 to 1023);
	public static int degreeToEncoder(double degree)
	{
		double res = (degree/359 * 1023);
		return (int) Math.round(res);
	}
	
	//Convert degrees (-180 to 180) to encoder counts (-512 to 512)
	public static double mapAngleToEnc(double angle)
	{
		if (angle>=0){
			return (angle - 0)*(1023-512)/(180 - 0) + 512;
		} 
		else {
			return (angle - -180)*(511 - 0)/(-1 - -180) + 0;
		}
		
	}
	
	//Convert encoder counts (-512 to 512) to degrees (-180 to 180);
	public static double mapEncToAngle(double enc)
	{
		if (enc >= 512){
			return (enc - 512)*(180-0)/(1023 - 512) + 0;
		} 
		else {
			return (enc - 0)*(0 - -180)/(511 - 0) + -180;
		}
	}
	
	//Convert joystick (-1 to 1) to degrees (-180 to 180);
	//TODO | Make this return something that makes more sense
	public static double mapStickToAngle(double stick)
	{
		return (stick - -1)*(180 - -180)/(1 - -1) + -180;
	}
	
	//Returns true if val is above upperBound and below lowerBound.
	//Swaps upperBound and lowerBound if upperBound < lowerBound.
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
	
	//Keeps given encoder count within range (0 to 1023)
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
	
	//Rotates a vector by an angle (theta)
	public static double[] rotateVector(double[] vector, double theta) {
		theta = Math.toRadians(theta);
		double x = vector[0];
		double y = vector[1];
		double rx = (x * Math.cos(theta)) - (y * Math.sin(theta));
		double ry = (x * Math.sin(theta)) + (y * Math.cos(theta));
		double[] result = new double[2];
		result[0] = rx;
		result[1] = ry;
		return result;
	}
}