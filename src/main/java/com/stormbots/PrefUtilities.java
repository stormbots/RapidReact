package com.stormbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PrefUtilities{

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static double getDouble(String key, double backup){
		if( Preferences.containsKey(key) ){
			return Preferences.getDouble(key, backup);
		}else{
			Preferences.initDouble(key, backup);
			return backup;
		}
	}

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static int getInt(String key, int backup){
		if( Preferences.containsKey(key) ){
			return Preferences.getInt(key, backup);
		}else{
			Preferences.initInt(key, backup);
			return backup;
		}
	}

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static boolean getBoolean(String key, boolean backup){
		if( Preferences.containsKey(key) ){
			return Preferences.getBoolean(key, backup);
		}else{
			Preferences.initBoolean(key, backup);
			return backup;
		}
	}
}