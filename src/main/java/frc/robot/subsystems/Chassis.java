// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;



/** Creates a new Chassis. */
public class Chassis extends SubsystemBase {

  /** Utility enum to provide named values for shifted gear states 
   * This is only mostly needed because otherwise, we get jank naming
  */
  public enum Gear{
    HIGH(true, true),
    LOW(false, false);
    private boolean compbot,practicebot;
    Gear(boolean compbot, boolean practicebot){
      this.compbot = compbot;
      this.practicebot = practicebot;
    }
    public boolean bool(){return Constants.botName == BotName.COMP ? this.compbot : this.practicebot;};
  }

  private CANSparkMax left;
  private CANSparkMax right;
  private CANSparkMax leftA;
  private CANSparkMax leftB;
  private CANSparkMax rightA;
  private CANSparkMax rightB;

  DifferentialDrive chassis;
  Solenoid shifter;
  private AHRS navx;
  
  public Chassis(AHRS navx) {
    this.navx = navx;

    //Instantiate motors.
    left = new CANSparkMax(1,MotorType.kBrushless);
    leftA = new CANSparkMax(2,MotorType.kBrushless);
    leftB = new CANSparkMax(3,MotorType.kBrushless);
    right = new CANSparkMax(4,MotorType.kBrushless);
    rightA = new CANSparkMax(5,MotorType.kBrushless);
    rightB = new CANSparkMax(6,MotorType.kBrushless);

    //loop through motors and set common parameters
    for(CANSparkMax m : new CANSparkMax[]{left,right}){
      m.setOpenLoopRampRate(0.2);
      m.setIdleMode(IdleMode.kBrake);
    }
    //configure followers
    for(CANSparkMax m : new CANSparkMax[]{leftA,leftB}){ m.follow(left); }
    for(CANSparkMax m : new CANSparkMax[]{rightA,rightB}){ m.follow(right); }


    //Set other non-common parameters for motors
    //TODO: Verify that only the lead motor needs to be inverted. This should be the case
    left.setInverted(true);
    right.setInverted(!left.getInverted());

    //Set up drive train and kinematics
    chassis = new DifferentialDrive(left,right);


    //Set up the shifter and solenoids
    shifter = new Solenoid(PneumaticsModuleType.REVPH, 1); //TODO: set to correct channel and/or module
    setGear(Gear.LOW);
  }

  public void arcadeDrive(double power, double turn) {
    chassis.arcadeDrive(power,turn);
  }
  
  public void tankDrive(double powerLeft, double powerRight) {
    chassis.tankDrive(powerLeft, powerRight);
  }

  public void setGear(Gear gear){
    shifter.set(gear.bool());
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
