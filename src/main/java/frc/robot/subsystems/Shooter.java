// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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

  public CANSparkMax topMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax bottomMotor = new CANSparkMax(13, MotorType.kBrushless);
  private RelativeEncoder encoderTop;
  private RelativeEncoder encoderBottom;
  private final SparkMaxPIDController pidTop;
  private final SparkMaxPIDController pidBottom;
  private double rpmSetpoint = 0;

  SlewRateLimiter rpmslew = new SlewRateLimiter(0);
  Vision vision;

  public Shooter(Vision vision) {
    SmartDashboard.putNumber("rpmSetpoint", 0.0);
    switch(Constants.botName){
      case COMP:
        topMotor.setInverted(true);
        bottomMotor.setInverted(false);
      break;
      case PRACTICE:
        topMotor.setInverted(true);
        bottomMotor.setInverted(false);
      break;
    }

    this.vision = vision;

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    topMotor.setSmartCurrentLimit(30);
    bottomMotor.setSmartCurrentLimit(30);

    pidTop = topMotor.getPIDController();
    pidBottom = bottomMotor.getPIDController();
    encoderTop = topMotor.getEncoder();
    encoderBottom = bottomMotor.getEncoder();

    pidTop.setP(kPTop);
    pidBottom.setP(kPBottom);

    //TEST:
    //pidTop.setOutputRange(0, 1);
    //pidBottom.setOutputRange(0, 1);

    rpmslew = new SlewRateLimiter(6000/1.0);
    pidTop.setFF(1.0/Constants.kNeoMaxRPM);
    pidBottom.setFF(1.0/Constants.kNeoMaxRPM);
    }
    
    public void setRPM(double rpmSetpoint){
      this.rpmSetpoint = rpmSetpoint;
    }

    //TODO: Command: While held, if has target, put distance into a variable. If aiming and target lost, use old distance
    //TODO: Automatically grabs distance from limelight:
    public void setRPMForDistance(double distance){
      rpmSetpoint = Constants.distanceToRPM.getOutputAt(distance);
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
    double output;
    double gonnaTryMathNow(){
      return output;

    }

  public void shooterSpoolUpToSpeed(){
    double kShooterSpeed = .53;

    topMotor.set(kShooterSpeed);
    bottomMotor.set(kShooterSpeed*.9);
  }

  public void shooterOff(){
    topMotor.set(0.0);
    bottomMotor.set(0.0);
  }

  @Override
  public void periodic() {

    //setRPM(SmartDashboard.getNumber("rpmSetpoint", 0.0));
    //Temporarily disabled so the motor values can be run manually on test code
    // pidTop.setReference(kShooterMotorRatio * rpmSetpoint, ControlType.kVelocity, 0);
    // pidBottom.setReference(rpmSetpoint, ControlType.kVelocity, 0);




    pidTop.setReference(rpmslew.calculate(rpmSetpoint) * kShooterMotorRatio, ControlType.kVelocity, kPIDSlot);
    pidBottom.setReference(rpmslew.calculate(rpmSetpoint), ControlType.kVelocity, kPIDSlot);
    SmartDashboard.putNumber("shooter/bottomamps", bottomMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/rpmBottom", getRPMBottom());
    SmartDashboard.putNumber("shooter.rpmTop", getRPMTop());
  }
}
