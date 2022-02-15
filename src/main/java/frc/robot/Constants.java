// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // put robot-wide constants and parameters here.
  // For subsystem specific constants, leave them in the relevant subsystem.
  // Everything in this class should be static, but making them final/const is optional
  public enum BotName {TABI, COMP, PRACTICE};
  public static BotName botName = BotName.COMP;

  //----------------------------------------------
  //----ALL DISTANCE UNITS MUST BE IN INCHES!!----
  //----------------------------------------------
  //-----(if not, harlod will hunt you down)------
  //----------------------------------------------

  public static double scalar;
  public static double sVolts;
  public static double vVoltSecondsPerInch;
  public static double aVoltSecondsSquaredPerInch;
  public static double PDriveVel;
  public static double TrackwidthInches;
  public static DifferentialDriveKinematics DriveKinematics;
  public static double EncoderDistancePerPulse;
  public static double MaxSpeedInchesPerSecond;
  public static double MaxAccelerationInchesPerSecondSquared;
  public static double RamseteB;
  public static double RamseteZeta;

  public static void Initialize() {
    // Called before robot subsystems are generated, and can safely set or re-set values
    // for any constants. Note, this is _not_ a constructor, since constants are static.
    switch (botName)
    {
      case TABI:
          // NEEDS TO BE RECHARACTERIZED IN INCHES
          scalar = 3.3;
          sVolts = 0.10362 * scalar;
          vVoltSecondsPerInch = 0.39906 * scalar;
          aVoltSecondsSquaredPerInch = 0.060961 * scalar;
          PDriveVel = 0.0039806;
          TrackwidthInches = 12;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthInches);
          EncoderDistancePerPulse = (14.0 / 72.0) * 4.0 * Math.PI; // Gearing * Wheel Diameter * PI = 0.0623
          MaxSpeedInchesPerSecond = 36;
          MaxAccelerationInchesPerSecondSquared = 36;
          RamseteB = 2 * 2;
          RamseteZeta = 0.7 * 4;
          break;
      case PRACTICE:
          scalar = 1.0;
          sVolts = 0.46035 * scalar;
          vVoltSecondsPerInch = 0.012599 * scalar;
          aVoltSecondsSquaredPerInch = 0.0099717 * scalar;
          PDriveVel = 0;//0.019794;
          TrackwidthInches = 32;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthInches);
          EncoderDistancePerPulse = (50.0 / 34.0) * 4.25 * Math.PI; // Gearing * Wheel Diameter * PI
          MaxSpeedInchesPerSecond = 12;
          MaxAccelerationInchesPerSecondSquared = 12;
          RamseteB = 2;
          RamseteZeta = 0.7;
          break;
      case COMP:
      default:
          // NEEDS TO BE UPDATED BEFORE RUNNING (remember harlod? yeah....)
          scalar = 1;
          sVolts = 1 * scalar;
          vVoltSecondsPerInch = 1 * scalar;
          aVoltSecondsSquaredPerInch = 1 * scalar;
          PDriveVel = 1;
          TrackwidthInches = 1;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthInches);
          EncoderDistancePerPulse = (50.0 / 34.0) * 4.25 * Math.PI; // Gearing * Wheel Diameter * PI
          MaxSpeedInchesPerSecond = 1;
          MaxAccelerationInchesPerSecondSquared = 1;
          RamseteB = 2;
          RamseteZeta = 0.7;
          break;
    }
  }
}
