// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {

  Chassis chassis;
  Vision vision;
  AHRS gyro;

  DoubleSupplier fwdPower;
  DoubleSupplier turnpower;

  /** Creates a new ChassisVisionTargeting. */
  public ChassisVisionTargeting(DoubleSupplier fwdPower, DoubleSupplier turnpower, Chassis chassis, Vision vision, AHRS gyro) {
    this.chassis = chassis;
    this.vision = vision;
    this.gyro = gyro;
    this.fwdPower = fwdPower;
    this.turnpower = turnpower;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.lightsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = fwdPower.getAsDouble();
    double outputTurn = turnpower.getAsDouble();

    //alternative driver locking method
    //in initialize, set found_target toi false
    //if we see a target, set found_target to true
    //found_target == true, no driver turning

    if (vision.hasValidTarget() == false){
      chassis.arcadeDrive(forward,outputTurn);
      return;
    }

    

    outputTurn = vision.pidTurn.getOutput(0, vision.getX());

    // outputTurn = vision.pidTurn.getOutput(gyro.getAngle(), vision.getTargetHeading());
    SmartDashboard.putNumber("vision/aimOutput", outputTurn);
    chassis.arcadeDrive(forward, outputTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
