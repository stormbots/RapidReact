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
  public enum BotName {COMP,PRACTICE};
  public static BotName botName = BotName.COMP;

  // TABI pathfinding constants
  public static final double scalar = 3.3;
  public static final double ksVolts = 0.10362 * scalar;
  public static final double kvVoltSecondsPerMeter = 0.39906 * scalar;
  public static final double kaVoltSecondsSquaredPerMeter = 0.060961 * scalar;
  public static final double kPDriveVel = 0.0039806;
  public static final double kTrackwidthMeters = 0.3;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kEncoderDistancePerPulse = (14.0 / 72.0) * 0.102 * Math.PI; // Gearing * Wheel Diameter (meters) * PI = 0.0623
  public static final double kMaxSpeedMetersPerSecond = 1; // Update Suggested
  public static final double kMaxAccelerationMetersPerSecondSquared = 1; // Update Suggested
  public static final double kRamseteB = 2 * 2;
  public static final double kRamseteZeta = 0.7 * 4;

  public static void Initialize() {
    // Called before robot subsystems are generated, and can safely set or re-set values
    // for any constants. Note, this is _not_ a constructor, since constants are static.
  }
}
