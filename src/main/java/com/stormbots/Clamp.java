package com.stormbots;

public class Clamp{
	public static double clamp(double x, double min, double max) {
		if(x < min) {
			return min;
		}
		if(x > max) {
			return max;
		}
		return x;
	}
	public static boolean bounded(double x, double min, double max) {
		if(x < min) {
			return false;
		}
		if(x > max) {
			return false;
		}
		return true;
	}
}