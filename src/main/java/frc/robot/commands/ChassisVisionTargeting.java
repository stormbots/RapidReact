// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {

  Chassis chassis;
  Vision vision;
  AHRS gyro;

  double outputTurn;
  /** Creates a new ChassisVisionTargeting. */
  public ChassisVisionTargeting(Chassis chassis, Vision vision, AHRS gyro) {
    this.chassis = chassis;
    this.vision = vision;
    this.gyro = gyro;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (vision.hasValidTarget()){
      chassis.arcadeDrive(0, 0);
      return;
    }

    outputTurn = vision.pidTurn.getOutput(gyro.getAngle(), vision.getTargetHeading());

    chassis.arcadeDrive(0, outputTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
