// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. 
   * @param canSparkMax
   */
  public CANSparkMax motor;
  double kIntakeSpeed = 0.8;
  Solenoid intakeSolenoid;
  boolean kUp;
  boolean kDown;
  
  public Intake(CANSparkMax motor, int solenoidChannel) {
    switch(Constants.botName){
      case COMP:
       motor.setInverted(false);
       intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidChannel);
      break;
      case PRACTICE:
        motor.setInverted(true);
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidChannel);
      break;
    }
    this.motor = motor;

   
    motor.setSmartCurrentLimit(30);

    kUp = false;
    kDown = true;
    
    intakeSolenoid.set(kUp);
  }

  public void intakeOn(){
    motor.set(kIntakeSpeed);
    intakeSolenoid.set(kDown);
  }
  public void intakeOff(){
    motor.set(0.0);
    intakeSolenoid.set(kUp);
  }
  public void intakeEject(){
    motor.set(-kIntakeSpeed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
