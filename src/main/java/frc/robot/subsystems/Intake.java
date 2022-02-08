// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. 
   * @param canSparkMax
   */
  public CANSparkMax motor;
  double kIntakeSpeed = 0.3;

  public Intake(CANSparkMax motor) {
    this.motor = motor;

    motor.setInverted(true);
    motor.setSmartCurrentLimit(30);

    switch(Constants.botName){
      case PRACTICE:
      break;
      case COMP:
      }
  }

  public void intakeOn(){
    motor.set(kIntakeSpeed);
  }
  public void intakeOff(){
    motor.set(0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
