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
  //----ALL DISTANCE UNITS MUST BE IN METERS!!----
  //----------------------------------------------
  //-----(if not, harlod will hunt you down)------
  //----------------------------------------------

  public static double scalar;
  public static double sVolts;
  public static double vVoltSecondsPerMeter;
  public static double aVoltSecondsSquaredPerMeter;
  public static double PDriveVel;
  public static double TrackwidthMeters;
  public static DifferentialDriveKinematics DriveKinematics;
  public static double EncoderDistancePerPulse;
  public static double MaxSpeedMetersPerSecond;
  public static double MaxAccelerationMetersPerSecondSquared;
  public static double RamseteB;
  public static double RamseteZeta;

  public static void Initialize() {
    // Called before robot subsystems are generated, and can safely set or re-set values
    // for any constants. Note, this is _not_ a constructor, since constants are static.
    switch (botName)
    {
      case TABI:
          // Possibly needs to be recharacterized
          scalar = 3.3;
          sVolts = 0.10362 * scalar;
          vVoltSecondsPerMeter = 0.39906 * scalar;
          aVoltSecondsSquaredPerMeter = 0.060961 * scalar;
          PDriveVel = 0.0039806;
          TrackwidthMeters = 0.3;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
          EncoderDistancePerPulse = (14.0 / 72.0) * 0.1016 * Math.PI; // Gearing * Wheel Diameter * PI = 0.0623
          MaxSpeedMetersPerSecond = 3;
          MaxAccelerationMetersPerSecondSquared = 3;
          RamseteB = 2 * 2;
          RamseteZeta = 0.7 * 4;
          break;
      case PRACTICE:
          // Field test -> 10 ft return 35.8 encoder counts on left main
          // 11.7 -> 1 meter = 0.085
          scalar = 1.0;
          sVolts = 0.32829 * scalar;
          vVoltSecondsPerMeter = 0.55442 * scalar;
          aVoltSecondsSquaredPerMeter = 0.15908 * scalar;
          PDriveVel = 0.80858;
          TrackwidthMeters = 0.8128;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
          EncoderDistancePerPulse = (12.0 / 50.0) * 0.10795 * Math.PI; // Gearing * Wheel Diameter * PI = 0.08139
          MaxSpeedMetersPerSecond = 1;
          MaxAccelerationMetersPerSecondSquared = 1;
          RamseteB = 2 * 2;
          RamseteZeta = 0.7 * 4;
          break;
      case COMP:
      default:
          // NEEDS TO BE UPDATED BEFORE RUNNING (remember harlod? yeah....)
          scalar = 1;
          sVolts = 1 * scalar;
          vVoltSecondsPerMeter = 1 * scalar;
          aVoltSecondsSquaredPerMeter = 1 * scalar;
          PDriveVel = 1;
          TrackwidthMeters = 1;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
          EncoderDistancePerPulse = (12.0 / 50.0) * 0.10795 * Math.PI; // Gearing * Wheel Diameter * PI
          MaxSpeedMetersPerSecond = 1;
          MaxAccelerationMetersPerSecondSquared = 1;
          RamseteB = 2;
          RamseteZeta = 0.7;
          break;
    }
  }
}
