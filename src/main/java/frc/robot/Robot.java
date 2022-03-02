// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BotName;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    String botString = Preferences.getString("botName", "none").toUpperCase().trim();
    switch(botString){
      case "COMP":     Constants.botName = BotName.COMP;break;
      case "PRACTICE": Constants.botName = BotName.PRACTICE;break;
      default:
        botString = "COMP";
        Constants.botName = BotName.COMP;
        System.err.println("ROBOT NAME NOT DEFINED:");
        System.err.println("Assuming COMP for safety: View Preferences to change");
    }
    Preferences.setString("botName",botString);
    SmartDashboard.setPersistent("Preferences/botName");

    //We may have different constants between the two robots
    //At this stage, we can safely configure those differences
    Constants.Initialize();


    // Instantiate our RobotContainer.  This will perform all our button bindings, create subsystems, 
    // and put our, and put autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.vision.lightsOff();
    m_robotContainer.climber.winchMotor.setIdleMode(IdleMode.kCoast);
    m_robotContainer.climber.hookMotor.set(0.0);
    m_robotContainer.climber.winchMotor.set(0.0);
    m_robotContainer.chassis.setIdleMode(IdleMode.kCoast);
    }

  @Override
  public void disabledPeriodic() {
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.chassis.setIdleMode(IdleMode.kBrake);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.passthrough.ptIncrementCargo();//we start auto with 1
    m_robotContainer.climber.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.chassis.setIdleMode(IdleMode.kBrake);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.vision.lightsOn();
    m_robotContainer.climber.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("chassis/turn", m_robotContainer.driver.getRawAxis(2));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
