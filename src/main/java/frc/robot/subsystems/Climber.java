// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  //Class consents including the min height and max height
  private double kMinHeight = 45;
  private double kMaxHeight = 61;

  //Class consent variables for the motor for the hookMotor and climber, also gets the encoder for the hook and the PID controller
  public CANSparkMax hookMotor = new CANSparkMax(16, MotorType.kBrushless); //Change the port later, 10 is temp variable
  private RelativeEncoder hookEncoder = hookMotor.getEncoder();

  private SparkMaxPIDController hookPID = hookMotor.getPIDController();
  public CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless); //15
  private RelativeEncoder winchEncoder = winchMotor.getEncoder();
  MiniPID winchPID = new MiniPID(0,0,0);//TODO: change temp values. 

  // SlewRateLimiter
  SlewRateLimiter winchSlewRate = new SlewRateLimiter(1); //TO-DO; change the name later and temp value of 1\
  
  //Note to self; motorHook switched to hookMotor and motorCLimber switched to winchMotor

  //Switch imports
  //Note: this may not be added or used
  DigitalInput homeSwitch = new DigitalInput(7); //Change the port later, 7 is temp variable


  /** Creates a new Climber. */
  public Climber() {
    //Get the position of the encoder
    hookEncoder.getPosition();

    //configure climber hook/motors
      //configure conversion factor for encoder so that X rotations  = linear distance
      //configure offset for that value
      winchMotor.setInverted(true);
      winchEncoder.setPositionConversionFactor((kMaxHeight-kMinHeight)/136.25); //TODO set conversion to inches from floor;
      winchEncoder.setPosition(kMinHeight); //TODO Set to resting height from floor
      
      winchMotor.setSoftLimit(SoftLimitDirection.kForward, (float)kMaxHeight);
      winchMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)kMinHeight);
      // winchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      // winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      
      winchPID.setSetpoint(winchEncoder.getPosition());
      winchMotor.setIdleMode(IdleMode.kBrake);//Want coast on boot, enable brake when climbing
      winchMotor.setSmartCurrentLimit(80);
      //Configure Hook
      hookMotor.setSmartCurrentLimit(15);
      hookMotor.setInverted(false);
      hookEncoder.setPositionConversionFactor(180.0/306.5); //306.5
      hookEncoder.setPosition(0);

      
      hookMotor.setSoftLimit(SoftLimitDirection.kForward, 180);
      hookMotor.setSoftLimit(SoftLimitDirection.kReverse, 45);
      hookMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      hookMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      
      // hookPID.setReference(hookEncoder.getPosition(), ControlType.kPosition);
      hookPID.setP(1/360.0);
      hookMotor.setIdleMode(IdleMode.kCoast);
      hookMotor.set(0);
  } 

  // getHeight()
  public double getHeight(){
   return winchEncoder.getPosition(); 
  }
  public void setHeight(double targetHeight){
    winchPID.setSetpoint(targetHeight);
  }

  public void setWinchClimbFF(boolean feedforward){
    //TODO: Test system properly, then add these
    // if(feedforward){ winchPID.setF( (s,a,e)-> -0.1 ); }
    // else{ winchPID.setF( (s,a,e)-> 0 ); }
  }
  //Power to the hook so it can hook on
  public void setHookAngle(double angle) {
    //get a distance
    //once distance reached stop
    //hookPID.setReference(angle, ControlType.kPosition);
  }

  public void setWinchPower(double power){
    // power = Clamp.clamp(power, -.3, .3);
    // power *=0.3;
    winchMotor.set(power);
  }

  
  public void init(){
    winchMotor.setIdleMode(IdleMode.kBrake);
    hookMotor.setIdleMode(IdleMode.kBrake);

  }

  public void disabledInit(){
    // winchMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // Use the SlewRateLimiter
    double power = winchPID.getOutput(getHeight());
    // winchMotor.set(power);

    // set a pid config and setup for the hook
    // double hookMotorPower = hookPID.setReference(value, ControlType.kPosition);
    
    // print target, current values, and outputs for all pid systems
    // Ex;SmartDashboard.putString("climber/whatever", value);
    //Encoder outputs
    //Target Outputs
    //SmartDashboard.putNumber("climber/target", value);
    // //PID Outputs
    SmartDashboard.putNumber("climber/winch/amps", winchMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber/winch/output", winchMotor.getAppliedOutput());
    SmartDashboard.putNumber("climber/winch/position", winchMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("climber/hook/amps", hookMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber/hook/output", hookMotor.getAppliedOutput());    
    SmartDashboard.putNumber("climber/hook/position", hookMotor.getEncoder().getPosition());
    
  }

}
