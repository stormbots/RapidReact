// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;


/** Creates a new Chassis. */
public class Chassis extends SubsystemBase {

  //----------------------------------------------
  //----ALL DISTANCE UNITS MUST BE IN METERS!!----
  //----------------------------------------------
  //-----(if not, harlod will hunt you down)------
  //----------------------------------------------

  public static double scalar;
  public static double sVolts;
  public static double vVoltSecondsPerMeter;
  public static double aVoltSecondsSquaredPerMeter;
  public static double PDriveVel;
  public static double TrackwidthMeters;
  public static DifferentialDriveKinematics DriveKinematics;
  public static double EncoderDistancePerPulse;
  public static double MaxSpeedMetersPerSecond;
  public static double MaxAccelerationMetersPerSecondSquared;
  public static double RamseteB;
  public static double RamseteZeta;

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
  
  public boolean reverse = false;

  DifferentialDrive chassis;
  Solenoid shifter;
  private AHRS navx;
  
  public Chassis(AHRS navX) {
    
    
    this.navx = navX;
    navx.reset();
    navx.calibrate();

    odometry = new DifferentialDriveOdometry(navx.getRotation2d());

    //Adjust various motor constants
    switch (Constants.botName)
    {
      case PRACTICE:
          // Field test -> 10 ft return 35.8 encoder counts on left main
          // 11.7 -> 1 meter = 0.085
          scalar = 1.0;
          sVolts = 0.32829 * scalar;
          vVoltSecondsPerMeter = 0.55442 * scalar;
          aVoltSecondsSquaredPerMeter = 0.15908 * scalar;
          PDriveVel = 0.80858;
          TrackwidthMeters = 0.8128;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
          EncoderDistancePerPulse = (12.0 / 50.0) * 0.10795 * Math.PI; // Gearing * Wheel Diameter * PI = 0.08139
          MaxSpeedMetersPerSecond = 1.5; // Doesn't work with pathweaver trajectories
          MaxAccelerationMetersPerSecondSquared = 1.5; // Doesn't work with pathweaver trajectories
          RamseteB = 2 * 2;
          RamseteZeta = 0.7 * 4;
          break;
      default:
      case COMP:
          // 127.7 rotations per 120 in
          // NEEDS TO BE UPDATED BEFORE RUNNING (remember harlod? yeah....)
          scalar = 1.0;
          sVolts = 0.17731 * scalar;
          vVoltSecondsPerMeter = 0.96673 * scalar;
          aVoltSecondsSquaredPerMeter = 0.080129 * scalar;
          PDriveVel = 1.0978;
          TrackwidthMeters = 0.68;
          DriveKinematics = new DifferentialDriveKinematics(TrackwidthMeters);
          EncoderDistancePerPulse = (12.0 / 50.0) * 0.10795 * Math.PI; // Gearing * Wheel Diameter * PI
          MaxSpeedMetersPerSecond = 4; // Doesn't work with pathweaver trajectories
          MaxAccelerationMetersPerSecondSquared = 4; // Doesn't work with pathweaver trajectories
          RamseteB = 2 * 2;
          RamseteZeta = 0.7 * 0.4;
          break;
    }

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

    leftEncoder.setPositionConversionFactor(Chassis.EncoderDistancePerPulse);
    leftEncoder.setVelocityConversionFactor(Chassis.EncoderDistancePerPulse / 60); // RPM to m/s
    rightEncoder.setPositionConversionFactor(Chassis.EncoderDistancePerPulse);
    rightEncoder.setVelocityConversionFactor(Chassis.EncoderDistancePerPulse / 60); // RPM to m/s

    // leftEncoder.setPositionConversionFactor(1.0);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //loop through motors and set common parameters
    for(CANSparkMax m : new CANSparkMax[]{left,right,leftA,rightA,leftB,rightB}){
      m.setOpenLoopRampRate(0.2);
      m.setIdleMode(IdleMode.kBrake);
      m.setSmartCurrentLimit(240/6, 240/6*2);//240 is sensible current limit to chassis
    }
    //configure followers
    for(CANSparkMax m : new CANSparkMax[]{leftA,leftB}){
      m.follow(left);
    }
    for(CANSparkMax m : new CANSparkMax[]{rightA,rightB}){
      m.follow(right);
    }

    //Set other non-common parameters for motors
    left.setInverted(true);
    right.setInverted(!left.getInverted());

    //Set up drive train and kinematics
    chassis = new DifferentialDrive(left,right);

    //Set up the shifter and solenoids
    switch(Constants.botName){
      case COMP:
        shifter = new Solenoid(PneumaticsModuleType.REVPH, 4);
      break;
      case PRACTICE:
        shifter = new Solenoid(PneumaticsModuleType.REVPH, 1);
      break;
    }
    setGear(Gear.LOW);
  }

  private SlewRateLimiter forwardSlew = new SlewRateLimiter(1/.35);

  public void arcadeDrive(double power, double turn) {
    power = forwardSlew.calculate(power);
    chassis.arcadeDrive(power,turn);
  }
  
  public void tankDrive(double powerLeft, double powerRight) {
    chassis.tankDrive(powerLeft, powerRight);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    if (reverse) {
      left.setVoltage(-rightVolts);
      right.setVoltage(-leftVolts);
    } else {
      left.setVoltage(leftVolts);
      right.setVoltage(rightVolts);
    }
    chassis.feed();
  }

  public void resetOdometry(Pose2d pose)
  {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    if (reverse) {
      return new DifferentialDriveWheelSpeeds(-rightEncoder.getVelocity(), -leftEncoder.getVelocity());
    } else {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }
  }
  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void setGear(Gear gear){
    shifter.set(gear.bool());
  }

  public double getAverageDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }

  public void setIdleMode(IdleMode mode){
    for(CANSparkMax m : new CANSparkMax[]{left,right,leftA,rightA,leftB,rightB}){
      m.setIdleMode(mode);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left", leftEncoder.getPosition());
    if (left != null){
      SmartDashboard.putNumber("chassis/faults",left.getFaults());
    }
    if (reverse) {
      odometry.update(navx.getRotation2d(), -rightEncoder.getPosition(), -leftEncoder.getPosition());
    } else {
      odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    SmartDashboard.putNumber("chassis/x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("chassis/y", odometry.getPoseMeters().getY());
  }
}
