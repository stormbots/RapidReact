// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotorTop = new CANSparkMax(13,MotorType.kBrushless);
  public CANSparkMax shooterMotorBottom = new CANSparkMax(14,MotorType.kBrushless);
  double kShooterSpeed;

  public Shooter() {
    switch(Constants.botName){
      case PRACTICE:
      break;
      case COMP:
      }
      
      //Set Inversions TODO Tune Inversions
      shooterMotorTop.setInverted(false);
      shooterMotorBottom.setInverted(true);

      //Set Current Limits TODO Currently arbitrary Increase/Decrease as needed
      shooterMotorTop.setSmartCurrentLimit(30);
      shooterMotorBottom.setSmartCurrentLimit(30);
      
      kShooterSpeed = 0.85;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
