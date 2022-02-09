// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.stormbots.Clamp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**  Ratio of RPM for Top Motor : Bottom Motor */
  private final double kShooterMotorRatio = .9;
  private final double kRPMTargetBound = 100;

  private CANSparkMax topMotor = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
  private RelativeEncoder encoderTop;
  private RelativeEncoder encoderBottom;
  private final SparkMaxPIDController pidTop;
  private final SparkMaxPIDController pidBottom;
  private double rpmSetpoint = 0;

  public Shooter() {

    switch(Constants.botName){
      case COMP:
      break;

      case PRACTICE:
      default:
      break;
    }

    pidTop = topMotor.getPIDController();
    pidBottom = bottomMotor.getPIDController();
    encoderTop = topMotor.getEncoder();
    encoderBottom = bottomMotor.getEncoder();

    // TODO: configure PID values for Shooter Motors
    pidTop.setP(1.6233E-07);
    pidTop.setI(0);
    pidTop.setD(0);
    //pidTop.setFF(1/(5850 * (14/72)));

    pidBottom.setP(1.6233E-07);
    pidBottom.setI(0);
    pidBottom.setD(0);
    //pidBottom.setFF(1/(5850 * (14/72)));

    pidTop.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // TODO: configure SmartMotion values
    pidTop.setSmartMotionMaxAccel(1, 0);
    pidTop.setSmartMotionMaxVelocity(1000, 0);
    pidBottom.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    pidBottom.setSmartMotionMaxAccel(1, 0);
    pidBottom.setSmartMotionMaxVelocity(1000, 0);
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

    pidTop.setReference(kShooterMotorRatio * rpmSetpoint, ControlType.kVelocity, 0);
    pidBottom.setReference(rpmSetpoint, ControlType.kVelocity, 0);




    SmartDashboard.putNumber("shooter/rpmBottom", getRPMBottom());
    SmartDashboard.putNumber("shooter.rpmTop", getRPMTop());
  }
}
