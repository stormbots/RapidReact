// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;

public class CargoColorSensor extends SubsystemBase {
  I2C.Port colorPort; 
  Rev2mDistanceSensor.Port distancePort;
  
  private ColorSensorV3 colorSensor; 
  private Rev2mDistanceSensor distanceSensor; 

  public enum CargoColor {BLUE, RED, UNDEFINED};
  private CargoColor teamColor = CargoColor.RED; //temp value
  private Color color = Color.kGreen; //temp value
  
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.152, 0.390, 0.457);
  private final Color kRedTarget = new Color(0.543, 0.338, 0.118);
  ColorMatchResult match = colorMatcher.matchClosestColor(color);

  /** Creates a new CargoColorSensor. */
  public CargoColorSensor(I2C.Port colorPort, Rev2mDistanceSensor.Port distancePort) {
    // NOTE: just future proofing, don't worry about this quite yet.
    // just declare things how you normally do I think.
    this.colorPort = colorPort;
    this.distancePort = distancePort;

    colorSensor = new ColorSensorV3(colorPort);
    distanceSensor = new Rev2mDistanceSensor(distancePort);
    
    distanceSensor.setAutomaticMode(true);

    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kBlueTarget);

    switch (DriverStation.getAlliance()) {
      case Red:
        teamColor = CargoColor.RED;
        break;
      case Blue:
        teamColor = CargoColor.BLUE;
        break;
      default:
        break;
    }
  }

  public CargoColor getTeamColor(){
    return teamColor;
  }

  public CargoColor getColor() {
    color = colorSensor.getColor();
    match = colorMatcher.matchClosestColor(color);

    if(distanceSensor.getRange() > 1) return CargoColor.UNDEFINED;
    else if(distanceSensor.getRange() < 0) return CargoColor.UNDEFINED;

    else if (match.confidence <= .95) {
      SmartDashboard.putString("ColorSensor/color", "undefined");
      return CargoColor.UNDEFINED;
    }
    
    else if (match.color == kRedTarget) {
      SmartDashboard.putString("ColorSensor/color", "red");
      return CargoColor.RED;
    } 
    else if (match.color == kBlueTarget) {
      SmartDashboard.putString("ColorSensor/color", "blue");
      return CargoColor.BLUE;
    } 
    else{
      return CargoColor.UNDEFINED;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ColorSensor/red", colorSensor.getColor().red);
    SmartDashboard.putNumber("ColorSensor/green", colorSensor.getColor().green);
    SmartDashboard.putNumber("ColorSensor/blue", colorSensor.getColor().blue);
    SmartDashboard.putNumber("ColorSensor/confidence", match.confidence);
    SmartDashboard.putNumber("DistanceSensor/distance", distanceSensor.getRange());
  }
}
