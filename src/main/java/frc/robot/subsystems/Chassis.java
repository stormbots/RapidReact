// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;

  public DifferentialDriveOdometry odometry;
  
  DifferentialDrive chassis;
  Solenoid shifter;
  private AHRS navx;
  
  public Chassis(AHRS navX) {
    this.navx = navX;
    navx.calibrate();
    navx.reset();

    odometry = new DifferentialDriveOdometry(navx.getRotation2d());

    //Instantiate motors.
    left = new CANSparkMax(1,MotorType.kBrushless);
    leftA = new CANSparkMax(2,MotorType.kBrushless);
    leftB = new CANSparkMax(3,MotorType.kBrushless);
    right = new CANSparkMax(4,MotorType.kBrushless);
    rightA = new CANSparkMax(5,MotorType.kBrushless);
    rightB = new CANSparkMax(6,MotorType.kBrushless);

    //Get encoder references and apply conversions
    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();

    leftEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulse);
    leftEncoder.setVelocityConversionFactor(Constants.kEncoderDistancePerPulse / 60);
    rightEncoder.setPositionConversionFactor(Constants.kEncoderDistancePerPulse);
    rightEncoder.setVelocityConversionFactor(Constants.kEncoderDistancePerPulse / 60);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

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

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    chassis.feed();
  }

  public void resetOdometry(Pose2d pose)
  {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }
  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void setGear(Gear gear){
    shifter.set(gear.bool());
  }

  @Override
  public void periodic() {
    odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
