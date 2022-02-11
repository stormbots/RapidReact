// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
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
  public static double ksVolts;
  public static double kvVoltSecondsPerMeter;
  public static double kaVoltSecondsSquaredPerMeter;
  public static double kPDriveVel;
  public static double kTrackwidthMeters;
  public static DifferentialDriveKinematics kDriveKinematics;
  public static double kEncoderDistancePerPulse;
  public static double kMaxSpeedMetersPerSecond;
  public static double kMaxAccelerationMetersPerSecondSquared;
  public static double kRamseteB;
  public static double kRamseteZeta;

  public static void Initialize() {
    // Called before robot subsystems are generated, and can safely set or re-set values
    // for any constants. Note, this is _not_ a constructor, since constants are static.
    switch (botName)
    {
      case TABI:
          scalar = 3.3;
          ksVolts = 0.10362 * scalar;
          kvVoltSecondsPerMeter = 0.39906 * scalar;
          kaVoltSecondsSquaredPerMeter = 0.060961 * scalar;
          kPDriveVel = 0.0039806;
          kTrackwidthMeters = 0.3;
          kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
          kEncoderDistancePerPulse = (14.0 / 72.0) * 0.102 * Math.PI; // Gearing * Wheel Diameter * PI = 0.0623
          kMaxSpeedMetersPerSecond = 1;
          kMaxAccelerationMetersPerSecondSquared = 1;
          kRamseteB = 2 * 2;
          kRamseteZeta = 0.7 * 4;
          break;
      case PRACTICE:
          // I'll put some stuff here when we actually HAVE a robot
          break;
      case COMP:
      default:
          // Ditto
    }
  }
}
