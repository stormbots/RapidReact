// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.stormbots.devices.BlinkenPattern;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  Spark ledModule = new Spark(0);
  public LED() {

    switch(DriverStation.getAlliance()){
      case Blue: 
      ledModule.set(.83);
      break;
      case Red:
      ledModule.set(.61);
      break;
      default:
      }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
