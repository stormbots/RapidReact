package com.stormbots.closedloop;

/** 
	 * A spring-like closed loop system, that tries to snap to a zero-error state
	 * based on the square root of the error. 
	 * <br/>
	 * This system is simple to tune, and fairly robust 
	 * within a wide range of system dynamics.
	 */
	public class FB {
	/////////////////////
	// Static Variant //
	///////////////////

	/**
	 * Provides closed loop to controls 
	 * @param target target position
	 * @param actual current position as measured by sensors
	 * @param k gain constant for corrected velocity
	 * @return output velocity
	 */
	public static double fb(double target, double actual, double k) {
		double output=0.0;
		if(target>actual) {
			output=k*Math.sqrt(target-actual);
		}
		else {
			output=-k*Math.sqrt(actual-target);
		}
		if(output>1.0)output=1.0;
		if(output<-1.0)output=-1.0;
		return output;
	}
	
}
