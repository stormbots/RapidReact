// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  
  public final double kCameraHeight = 38;
  public final double kCameraAngle = 23;
  public final double kUpperHubHeight = 101.625;

  private AHRS gyro;

  public MiniPID pidTurn;

  /** Creates a new Limelight. */
  public Vision(AHRS gyro) {

    table = NetworkTableInstance.getDefault().getTable("limelight");


    //TODO: Dynamic resolution change as robot gets farther from target
    driverPipeline();
    //lightsOff();
    lightsOn();

    this.gyro = gyro;

    //TODO: This was stolen from infinite recharge
    pidTurn = new MiniPID(0,0,0);
    pidTurn.setSetpointRange(15); 
    pidTurn.setP(0.013*.5);
    //pidTurn.setI(0.001);
    pidTurn.setMaxIOutput(0.15);
    pidTurn.setOutputLimits(0.7);
    // pidTurn.setOutputRampRate(0.7/200.0);
    pidTurn.setF((s,a,e)->{return Math.signum(e)*0.30;/*static FeedForward*/ });
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

  public double distanceCache = 0;

  @Override
  public void periodic() {
    hasTargets = tv.getDouble(0) == 1 ? true : false;
    x = tx.getDouble(0);
    y = ty.getDouble(0);
    targetArea = ta.getDouble(0);
    skew = ts.getDouble(0);

    //Protip" Don't use "limelight" for your output table, because that's the input table
    // SmartDashboard.putBoolean("vision/HasTargets", hasTargets);
    // SmartDashboard.putNumber("vision/X", getX());
    // SmartDashboard.putNumber("vision/Y", y);
    // SmartDashboard.putNumber("vision/Area", targetArea);
    SmartDashboard.putNumber("vision/distancetohub(in)", getDistanceToUpperHub());


  }

  public boolean hasValidTarget(){
    return hasTargets;
  }
  public double getX(){
    return tx.getDouble(0.0);
  }
    /**
   * @returns Heading to target (0...360). Allows for going past discontinuity (-X...360++)
   */
  public double getTargetHeading() {
    return gyro.getAngle() + tx.getDouble(0.0);
  }
  public double getDistanceToUpperHub(){
    if(!hasValidTarget()) return (9*12);
    return((kUpperHubHeight - kCameraHeight)/(Math.tan(Math.toRadians(y + kCameraAngle))));
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
