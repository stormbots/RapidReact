// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Steps for characterization
// Run tests, put k values into a SimpleMotorFeedForward (wpi)
// feedForward.calculate(targetRPM/60, acceleration)
// Eliminte voltage spike by timing shooting acceleration then setting accelration to that
// pid.setReference(targetRPM, ControlType.kSmartVelocity, 0, feedForwardOutput, ArbFFunits.kVoltage);
// pid.setP(kP from characterization);
// 

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private enum ShooterAccelMethod{SMARTMOTION, SLEW};
  ShooterAccelMethod shooterAccelMethod = ShooterAccelMethod.SLEW;

  /**  Ratio of RPM for Top Motor : Bottom Motor */
  private final double kShooterMotorRatio = .9;
  private final double kRPMTargetBound = 100;
  private final int kPIDSlot = 0;
  // TODO: Gather values from characterization
  private final double kPTop = 0;
  private final double kSTop = 0.072423;
  private final double kVTop = 0.024549;
  private final double kATop = 0;
  private final double kPBottom = 0;
  private final double kSBottom = 0.072423;
  private final double kVBottom = 0.024549;
  private final double kABottom = 0;

  private CANSparkMax topMotor = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(14, MotorType.kBrushless);
  // private CANSparkMax topMotor = new CANSparkMax(1, MotorType.kBrushless);
  // private CANSparkMax bottomMotor = new CANSparkMax(4, MotorType.kBrushless);
  private RelativeEncoder encoderTop;
  private RelativeEncoder encoderBottom;
  private final SparkMaxPIDController pidTop;
  private final SparkMaxPIDController pidBottom;
  private double rpmSetpoint = 0;

  private SimpleMotorFeedforward feedForwardTop = new SimpleMotorFeedforward(kSTop, kVTop, kATop);
  private SimpleMotorFeedforward feedForwardBottom = new SimpleMotorFeedforward(kSBottom, kVBottom, kABottom);
  private double feedForwardOutputTop;
  private double feedForwardOutputBottom;

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

    bottomMotor.setInverted(true);

    pidTop = topMotor.getPIDController();
    pidBottom = bottomMotor.getPIDController();
    encoderTop = topMotor.getEncoder();
    encoderBottom = bottomMotor.getEncoder();

    pidTop.setP(kPTop);
    pidBottom.setP(kPBottom);

    switch(shooterAccelMethod){
      case SMARTMOTION:
        // TODO: configure SmartMotion values
        pidTop.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, kPIDSlot);
        pidTop.setSmartMotionMaxAccel(Constants.kNeoMaxRPM/60.0, kPIDSlot);
        pidTop.setSmartMotionMaxVelocity(Constants.kNeoMaxRPM, kPIDSlot);
        pidTop.setSmartMotionMinOutputVelocity(0, kPIDSlot);
        pidTop.setSmartMotionAllowedClosedLoopError(Constants.kNeoMaxRPM, kPIDSlot);

        pidBottom.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, kPIDSlot);
        pidBottom.setSmartMotionMaxAccel(Constants.kNeoMaxRPM/60.0, kPIDSlot);
        pidBottom.setSmartMotionMaxVelocity(Constants.kNeoMaxRPM, kPIDSlot);
        pidBottom.setSmartMotionMinOutputVelocity(0, kPIDSlot);
        pidBottom.setSmartMotionAllowedClosedLoopError(Constants.kNeoMaxRPM, kPIDSlot);
        break;

      case SLEW:
      rpmslew = new SlewRateLimiter(6000/1.0);
      }
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
    feedForwardOutputTop = feedForwardTop.calculate(kShooterMotorRatio*rpmSetpoint/60, Constants.kNeoMaxRPM/60.0) * 5;
    feedForwardOutputBottom = feedForwardBottom.calculate(rpmSetpoint/60, Constants.kNeoMaxRPM/60.0) * 5;

    switch(shooterAccelMethod){
      case SMARTMOTION:
        pidTop.setReference(kShooterMotorRatio * rpmSetpoint, ControlType.kSmartMotion, kPIDSlot, feedForwardOutputTop, ArbFFUnits.kVoltage);
        pidBottom.setReference(rpmSetpoint, ControlType.kSmartMotion, kPIDSlot, feedForwardOutputBottom, ArbFFUnits.kVoltage);
      case SLEW:
        pidTop.setReference(rpmslew.calculate(rpmSetpoint) * kShooterMotorRatio, ControlType.kVelocity, kPIDSlot, feedForwardOutputTop,ArbFFUnits.kVoltage);
        pidBottom.setReference(rpmslew.calculate(rpmSetpoint), ControlType.kVelocity, kPIDSlot, feedForwardOutputBottom, ArbFFUnits.kVoltage);
    }
    
    SmartDashboard.putNumber("shooter/rpmBottom", getRPMBottom());
    SmartDashboard.putNumber("shooter.rpmTop", getRPMTop());
  }
}
