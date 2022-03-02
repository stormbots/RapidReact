// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stormbots.PiecewiseLerp;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   //
   // put robot-wide constants and parameters here.
   // For subsystem specific constants, leave them in the relevant subsystem.
   // Everything in this class should be static, but making them final/const is optional

   //Don't change this: It's adjusted in Robot, based on the Robot Preferences saved on the Rio
   public enum BotName {COMP,PRACTICE};
   public static BotName botName = BotName.COMP;

  public static double kNeoMaxRPM = 5400.0;
   private double kChassisLength = 32;

  public static PiecewiseLerp distanceToRPM;
  public static void Initialize() {
    // Called before robot subsystems are generated, and can safely set or re-set values
    // for any constants. Note, this is _not_ a constructor, since constants are static.

    /* COMPBOT */
    if (botName==BotName.COMP){
      distanceToRPM = new PiecewiseLerp(
        new double[]{5*12, 6*12, 8*12, 10*12, 12*12, 14*12, 16*12, 18*12, 20*12, 22*12,22*12},
        new double[]{2500, 2500, 2650, 2850,  3200,  3150,  3250,  3400,  3550,  3650, 3650}
         );
    }
    /* PRACTICE */
    else{
      distanceToRPM = new PiecewiseLerp(
        new double[]{5*12, 6*12, 8*12, 10*12, 12*12, 14*12, 16*12, 18*12, 20*12, 22*12,},
        new double[]{2500, 2500, 2650, 2850,  3250,  3325,  3625,  3750,  3925,  4150}
         );
    }

  }
}
