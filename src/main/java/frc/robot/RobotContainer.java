// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveArcade;
import frc.robot.subsystems.CargoColorSensor;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  

  // 
  // Global sensors/sensor subsystems
  //
  public AHRS navx = new AHRS(Port.kMXP); // NOTE: Some prior years required usb for good performance. Port may change.
  public CargoColorSensor cargoColorSensor = new CargoColorSensor(I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
  public Vision vision = new Vision();
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  // 
  // SUBSYSTEMS
  //
  public Chassis chassis = new Chassis(navx);
  public Intake intake = new Intake(new CANSparkMax(19,MotorType.kBrushless));
  public Climber climber = new Climber();
  public Passthrough passthrough = new Passthrough();
  public Feeder feeder = new Feeder();
  public Shooter shooter = new Shooter();
  
  // 
  // ROBOT COMMAND DEFINITIONS
  //



  // 
  // JOYSTICK AND BUTTON ASSIGNMENTS
  //
  //Do not reassign ports in code: Always reassign  ports in your
  //local driver station to match these.
  public Joystick driver = new Joystick(0);
  public Joystick operator = new Joystick(1);

  //Other part of the test code
  public JoystickButton testclimbButton= new JoystickButton(operator, 7);


  // Used to communicate auto commands to dashboard.
  SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure our autonomous commands, and make sure drive team can select what they want

    autoChooser.setDefaultOption("Does nothing", new InstantCommand(()->{}));
    autoChooser.addOption("Also nothing", new InstantCommand(()->{}));
    SmartDashboard.putData("autos/autoSelection", autoChooser);


    //configure default commands
    chassis.setDefaultCommand(
      // this one's really basic, but needed to get systems moving right away.
      new RunCommand(
        ()->{chassis.arcadeDrive(-driver.getRawAxis(1),driver.getRawAxis(2));}
        ,chassis)
      );
      // new RunCommand(
      //   ()->{chassis.tankDrive(.25,.25);}
      //   ,chassis)
      // );

    //TODO This is climber test code, be careful
    testclimbButton.whileHeld(
      // this one's really basic, but needed to get systems moving right away.
      new RunCommand(
        ()->{climber.setWinchPower(-operator.getRawAxis(1)*0.5);}
        ,climber
        )
        );
    testclimbButton.whenReleased(new InstantCommand(()->climber.setWinchPower(0)));

    SmartDashboard.putData("climber/hook/setAngle0",new InstantCommand(()->{climber.setHookAngle(0);}));
    SmartDashboard.putData("climber/hook/setAngle180",new InstantCommand(()->{climber.setHookAngle(180);}));
    SmartDashboard.putData("climber/hook/setAngle90",new InstantCommand(()->{climber.setHookAngle(90);}));


    // Configure the button bindings
    configureButtonBindings();

    //Reset navx to make sure it's calibrated and zero'd at the robot's current heading
    navx.reset();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }


  public Command climbSequenceBlue = new InstantCommand(()->{})
    .andThen(new ChassisDriveArcade(chassis).withInterrupt(()->{return true /*if over line*/;}))
    .andThen(()->{/*command that drives X distance forward */})
    .andThen(()->{/*command that rotates to find sensor */})
    .andThen(()->{/*climber goes to position */})
    .andThen(()->{climber.setHookAngle(90);}) //Hook setup, 0 is a temp value
    .andThen(()->{climber.setHeight(45);}) //Have the climber go up
    .andThen(new RunCommand(()->{}).withInterrupt(()->climber.getHeight()>44))
    .andThen(()->{climber.setHookAngle(0); /*latch to bar? if seperate process */})
    .andThen(()->{/*climber.setClimbFeedForward();*/})
    .andThen(()->{climber.setHeight(35);}) //Have the climber go down
    ;

}
