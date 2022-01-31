package com.stormbots;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PrefUtilities{

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static double getDouble(String key, double backup){
		if( Preferences.getInstance().containsKey(key) ){
			return Preferences.getInstance().getDouble(key, backup);
		}else{
			Preferences.getInstance().putDouble(key, backup);
			return backup;
		}
	}

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static int getInt(String key, int backup){
		if( Preferences.getInstance().containsKey(key) ){
			return Preferences.getInstance().getInt(key, backup);
		}else{
			Preferences.getInstance().putInt(key, backup);
			return backup;
		}
	}

	/**
	 * Shortcut for handling Preference values on the SmartDashboard.
	 */
	public static boolean getBoolean(String key, boolean backup){
		if( Preferences.getInstance().containsKey(key) ){
			return Preferences.getInstance().getBoolean(key, backup);
		}else{
			Preferences.getInstance().getBoolean(key, backup);
			return backup;
		}
	}
	

}