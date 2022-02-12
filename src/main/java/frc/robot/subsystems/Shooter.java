// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.stormbots.Clamp;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  /**  Ratio of RPM for Top Motor : Bottom Motor */
  private final double kShooterMotorRatio = .9;
  private final double kRPMTargetBound = 100;
  private final int kPIDSlot = 0;
  private final double kPTop = 0;//0.099591e-3;
  private final double kPBottom = 0;//0.094701e-3;

  private CANSparkMax topMotor = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
  // private CANSparkMax topMotor = new CANSparkMax(1, MotorType.kBrushless);
  // private CANSparkMax bottomMotor = new CANSparkMax(4, MotorType.kBrushless);
  private RelativeEncoder encoderTop;
  private RelativeEncoder encoderBottom;
  private final SparkMaxPIDController pidTop;
  private final SparkMaxPIDController pidBottom;
  private double rpmSetpoint = 0;

  SlewRateLimiter rpmslew = new SlewRateLimiter(0);

  public Shooter() {
    SmartDashboard.putNumber("rpmSetpoint", 0.0);
    switch(Constants.botName){
      case COMP:
      break;

      case PRACTICE:
      default:
      break;
    }

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    bottomMotor.setInverted(true);

    pidTop = topMotor.getPIDController();
    pidBottom = bottomMotor.getPIDController();
    encoderTop = topMotor.getEncoder();
    encoderBottom = bottomMotor.getEncoder();

    pidTop.setP(kPTop);
    pidBottom.setP(kPBottom);

    rpmslew = new SlewRateLimiter(6000/1.0);
    pidTop.setFF(1.0/Constants.kNeoMaxRPM);
    pidBottom.setFF(1.0/Constants.kNeoMaxRPM);
    }
    
    public void setRPM(double rpmSetpoint){
      this.rpmSetpoint = rpmSetpoint;
    }
    public void setRPMForDistance(double distanceIN){
      this.rpmSetpoint = Constants.distanceToRPM.getOutputAt(distanceIN);
    }
    public void setRPMLowerHub(){
      this.rpmSetpoint = 0;
    }
    public double getRPMTop(){
      return encoderTop.getVelocity();
    }
    public double getRPMBottom(){
      return encoderBottom.getVelocity();
    }
    boolean isOnTargetRPM(){
      return
      (Clamp.bounded(rpmSetpoint * kShooterMotorRatio, encoderTop.getVelocity()-kRPMTargetBound, encoderTop.getVelocity()+kRPMTargetBound) &&
       Clamp.bounded(rpmSetpoint, encoderBottom.getVelocity()-kRPMTargetBound, encoderBottom.getVelocity()+kRPMTargetBound));
    }

  @Override
  public void periodic() {

    setRPM(SmartDashboard.getNumber("rpmSetpoint", 0.0));

    pidTop.setReference(rpmslew.calculate(rpmSetpoint) * kShooterMotorRatio, ControlType.kVelocity, kPIDSlot);
    pidBottom.setReference(rpmslew.calculate(rpmSetpoint), ControlType.kVelocity, kPIDSlot);
 
    SmartDashboard.putNumber("shooter/rpmBottom", getRPMBottom());
    SmartDashboard.putNumber("shooter.rpmTop", getRPMTop());
  }
}
