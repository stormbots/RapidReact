// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoColorSensor extends SubsystemBase {
  I2C.Port port; 

  /** Creates a new CargoColorSensor. */
  public CargoColorSensor(I2C.Port port) {
    // NOTE: just future proofing, don't worry about this quite yet.
    // just declare things how you normally do I think.
    this.port = port;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
