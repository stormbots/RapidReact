// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;

import javax.sound.sampled.AudioFormat.Encoding;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.Clamp;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  //Class consents including the min height and max height
  private double kMinHeight = 6;
  private double kMaxHeight = 20;

  //Class consent variables for the motor for the hookMotor and climber, also gets the encoder for the hook and the PID controller
  private CANSparkMax hookMotor = new CANSparkMax(17, MotorType.kBrushless); //Change the port later, 10 is temp variable
  private RelativeEncoder hookEncoder = hookMotor.getEncoder();

  private SparkMaxPIDController hookPID = hookMotor.getPIDController();
  private CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless); //15
  private RelativeEncoder winchEncoder = winchMotor.getEncoder();
  MiniPID winchPID = new MiniPID(0,0,0);//TODO: change temp values. 

  //Note to self; motorHook switched to hookMotor and motorCLimber switched to winchMotor

  //Switch imports
  //Note: this may not be added or used
  DigitalInput homeSwitch = new DigitalInput(7); //Change the port later, 7 is temp variable


  /** Creates a new Climber. */
  public Climber() {
    //configure hook motors/encoder
    hookMotor.setSmartCurrentLimit(2);//Note: 2 is temp variable
    //Get the position of the encoder
    hookEncoder.getPosition();

    //configure climber hook/motors
      //configure conversion factor for encoder so that X rotations  = linear distance
      //configure offset for that value
      winchEncoder.setPositionConversionFactor(1); //TODO set conversion to inches from floor;
      winchEncoder.setPosition(0); //TODO Set to resting height from floor
      winchPID.setSetpoint(winchEncoder.getPosition());

      //Configure Hook
      hookMotor.setSmartCurrentLimit(5);
      hookEncoder.setPositionConversionFactor(1); //TODO set conversion to degres/angle
      hookPID.setReference(hookEncoder.getPosition(), ControlType.kPosition);
      hookPID.setP(0);
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
    hookPID.setReference(angle, ControlType.kPosition);
  }

  public void setWinchPower(double power){
    // power = Clamp.clamp(power, -.3, .3);
    // power *=0.3;
    winchMotor.set(power);
  }
  

  // SlewRateLimiter
  SlewRateLimiter winchSlewRate = new SlewRateLimiter(1); //TO-DO; change the name later and temp value of 1\

  @Override
  public void periodic() {
    // Use the SlewRateLimiter
    double power = winchPID.getOutput(getHeight());
    // winchMotor.set(power);

    // set a pid config and setup for the hook

    
    // print target, current values, and outputs for all pid systems
    // Ex;SmartDashboard.putString("climber/whatever", value);
    //Encoder outputs
    //Target Outputs
    //SmartDashboard.putNumber("climber/target", value);
    //PID Outputs
    SmartDashboard.putNumber("climber/winch/amps", winchMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber/winch/output", winchMotor.getAppliedOutput());
    SmartDashboard.putNumber("climber/winch/position", winchMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("climber/hook/amps", hookMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber/hook/output", hookMotor.getAppliedOutput());    
    SmartDashboard.putNumber("climber/hook/position", hookMotor.getEncoder().getPosition());
    
  }

}
