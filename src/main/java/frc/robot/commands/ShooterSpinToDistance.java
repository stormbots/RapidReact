// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShooterSpinToDistance extends CommandBase {
  /** Creates a new ShooterSpinToDistance. */
  Vision vision;
  Shooter shooter;
  private double distanceCache = 0;
  public ShooterSpinToDistance(Vision vision, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasValidTarget()){
      distanceCache = vision.getDistanceToUpperHub();
    }
    //MAYBE: Have distance slew rate
    shooter.setRPMForDistance(distanceCache);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    distanceCache = 0;
    shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
