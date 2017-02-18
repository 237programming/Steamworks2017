package org.usfirst.frc.team237.robot;

public final class MathStuff {
	
	//Convert encoder counts (0 to 1023) to degrees (0 to 359);
	public static double encoderToDegrees(double encoderCount)
	{
//		return (encoderCount/1023f * 359f);
		return map(map(encoderCount, 10, 890, 0, 1023), 0, 1023, 0, 359);
	}
	
	//Convert degrees (0 to 359) to encoder counts (0 to 1023);
	public static int degreeToEncoder(double degree)
	{
//		double res = (degree/359 * 1023f);
		double res =  map(degree, 0, 359, 0, 1023);
		return (int) Math.round(res);
	}
	
	//Convert degrees (-180 to 180) to encoder counts (-512 to 512)
	public static double mapAngleToEnc(double angle)
	{
//		if (angle>=0) {
//			return (angle - 0)*(1023-512)/(180 - 0) + 512;
//		} 
//		else {
//			return (angle - -180)*(511 - min)/(-1 - -180) + 0;
//		}
		
		if(angle >= 0) {
			double angleToEnc = map(angle, 0, 180, 440, 890);
			return map(angleToEnc, 440, 890, 512, 1023);
		}
		else {
			double angleToEnc = map(angle, -180, 0, 10, 440);
			return map(angleToEnc, 10, 440, 0, 511);
		}
	}
	
	//Convert encoder counts (-512 to 512) to degrees (-180 to 180);
	public static double mapEncToAngle(double enc)
	{
		if (enc >= 512) {
			double encToAngle = map(enc, 512, 1023, 440, 890);
			return map(encToAngle, 440, 890, 0, 180);
//			return (enc - 512)*(180-0)/(1023 - 512) + 0;
		} 
		else {
			double encToAngle = map(enc, 0, 511, 10, 440);
			return map(encToAngle, 10, 440, -180, 0);
//			return (enc - 0)*(0 - -180)/(511 - 0) + -180;
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
		if (map(enc, 10, 890, 0, 1023) > 1023)
		{
			enc -= map(1024, 0, 1024, 10, 890); 
		} else if (map(enc, 10, 890, 0, 1023) < 10) {
			enc += map(1024, 0, 1024, 10, 890);
		}
		return map(enc, 0, 1024, 10, 890);
	}
	
	public static double map(double val, double min0, double max0, double min1, double max1)
	{
		return (val - min0) * (max1 - min1) / (max0 - min0) + min0;
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