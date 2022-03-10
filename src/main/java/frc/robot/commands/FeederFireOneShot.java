// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederFireOneShot extends CommandBase {
  private Feeder feeder;
  private static double targetDistance = 0;
  private double totalDistance = 0;
  

  /** Creates a new PTFireOneShot. */
  public FeederFireOneShot(Feeder feeder) {
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (totalDistance > targetDistance){
      targetDistance = totalDistance;
    }
    targetDistance += 6;
    // feeder.encoderFeederBack.setPosition(0);
    // feeder.encoderFeederFront.setPosition(0);

    feeder.encoderFeederBack.setPositionConversionFactor(1/3.0);
    feeder.encoderFeederFront.setPositionConversionFactor(1/3.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("feeder/totalDistance", totalDistance);
    SmartDashboard.putNumber("feeder/targetDistance", targetDistance);
    feeder.motorFeederBack.setIdleMode(IdleMode.kBrake);
    feeder.motorFeederFront.setIdleMode(IdleMode.kBrake);
    feeder.feederRun();
    //feeder.motorFeederBack.set(.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.feederOff();
    feeder.motorFeederBack.setIdleMode(IdleMode.kCoast);
    feeder.motorFeederFront.setIdleMode(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    totalDistance = (feeder.encoderFeederBack.getPosition() + feeder.encoderFeederFront.getPosition()) / 2.0;
    if (totalDistance > targetDistance){
      end(false);
    }
    return false;
  }
}
