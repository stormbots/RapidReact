// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  
  public final double kCameraHeight = 30;
  public final double kCameraAngle = 25;
  public final double kUpperHubHeight = 101.625;


  /** Creates a new Limelight. */
  public Vision() {

    table = NetworkTableInstance.getDefault().getTable("limelight");

    driverPipeline();
    lightsOff();
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  // Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry tv = table.getEntry("tv");
  private boolean hasTargets;
  //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  NetworkTableEntry tx = table.getEntry("tx");
  /** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
  private double x;
  //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  NetworkTableEntry ty = table.getEntry("ty");
  /** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
  private double y;
  //Target Area (0% of image to 100% of image)
  NetworkTableEntry ta = table.getEntry("ta");
  private double targetArea;
  // Rotation or "Skew" of Target
  NetworkTableEntry ts = table.getEntry("ts");
  private double skew;


  @Override
  public void periodic() {
    hasTargets = tv.getDouble(0) == 1 ? true : false;
    x = tx.getDouble(0);
    y = ty.getDouble(0);
    targetArea = ta.getDouble(0);
    skew = ts.getDouble(0);

    //Protip" Don't use "limelight" for your output table, because that's the input table
    SmartDashboard.putBoolean("vision/HasTargets", hasTargets);
    SmartDashboard.putNumber("vision/X", x);
    SmartDashboard.putNumber("vision/Y", y);
    SmartDashboard.putNumber("vision/Area", targetArea);
    SmartDashboard.putNumber("vision/distancetohub(in)", getDistanceToUpperHub());


  }

  public boolean hasValidTarget(){
    return hasTargets;
  }
  public double getX(){
    return x;
  }
  public double getDistanceToUpperHub(){
    return ((kUpperHubHeight - kCameraHeight)/(Math.tan(Math.toRadians(y + kCameraAngle))));
  }
  public void lightsOn(){
     table.getEntry("ledMode").setNumber(3);
  }
  public void lightsOff(){
     table.getEntry("ledMode").setNumber(1);
  }
  private void driverPipeline(){
    table.getEntry("pipeline").setNumber(1);
  }
}
