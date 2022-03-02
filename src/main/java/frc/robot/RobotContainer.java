// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisDriveArcade;
import frc.robot.commands.ChassisPath;
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.FeederEjectCargo;
import frc.robot.commands.FeederShootCargo;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.PTLoadCargo;
import frc.robot.commands.PTMoveCargo;
import frc.robot.commands.ShooterSpoolUp;
import frc.robot.subsystems.CargoColorSensor;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CargoColorSensor.CargoColor;

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
  public CargoColorSensor cargoColorSensorFront;
  public CargoColorSensor cargoColorSensorBack;
  

  public Vision vision = new Vision(navx);
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  // 
  // SUBSYSTEMS
  //
  public Chassis chassis = new Chassis(navx);
  public Intake frontIntake = new Intake(new CANSparkMax(7,MotorType.kBrushless),3);
  public Intake backIntake = new Intake(new CANSparkMax(8,MotorType.kBrushless), 2);
  public Climber climber = new Climber();
  public Passthrough passthrough = new Passthrough();
  public Feeder feeder = new Feeder();
  public Shooter shooter = new Shooter(vision);
  
  // 
  // ROBOT COMMAND DEFINITIONS
  //

  // 
  // JOYSTICK AND BUTTON ASSIGNMENTS
  //
  //Do not reassign ports in code: Always reassign  ports in your
  //local driver station to match these.
  public Joystick driver = new Joystick(0);
  JoystickButton shiftButton = new JoystickButton(driver, 7);
  JoystickButton aimButton = new JoystickButton(driver, 8);

  public Joystick operator = new Joystick(1);
  JoystickButton ejectBackButton = new JoystickButton(operator, 6);
  JoystickButton ejectFrontButton = new JoystickButton(operator, 5);
  JoystickButton intakeFrontButton = new JoystickButton(operator, 4);
  JoystickButton intakeBackButton = new JoystickButton(operator, 2);
  JoystickButton shootButton = new JoystickButton(operator, 1);
  JoystickButton spoolShooterButton = new JoystickButton(operator, 8);
  JoystickButton climbButtonManual = new JoystickButton(operator, 7);
  

  // Used to communicate auto commands to dashboard.
  SendableChooser<Command> autoChooser = new SendableChooser<>();


  private ChassisVisionTargeting chassisVisionTargeting = new ChassisVisionTargeting(()->0,()->0,chassis, vision, navx);
  
  Trigger ejectCargoOutBack;
  Trigger ejectCargoOutFront;
  
  Trigger loadCargoFrontIntake;
  Trigger loadCargoFromBack;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Command testAuto = new InstantCommand(()->{})
      .andThen(new ChassisPath(chassis, "Test 1", false))
      .andThen(new ChassisPath(chassis, "Test 1", true))
    ;

    autoChooser.setDefaultOption("Do nothing", new InstantCommand(()->{}));
    autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("autos/autoSelection", autoChooser);
    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    //Configure our autonomous commands, and make sure drive team can select what they want
    switch(Constants.botName){
    case COMP:
     cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kMXP);
     cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
    break;
    case PRACTICE:
     cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kOnboard , Rev2mDistanceSensor.Port.kMXP);
     cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kOnboard);
    break;
    }

    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    // compressor.disable();
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
    // testclimbButton.whileHeld(
    //   //this one's really basic, but needed to get systems moving right away.
    //   new RunCommand(
    //     ()->{climber.setWinchPower(-operator.getRawAxis(1)*0.5);}
    //     ,climber
    //     )
    //     );
    // testclimbButton.whenReleased(new InstantCommand(()->climber.setWinchPower(0)));

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

    aimButton.whileHeld(new ChassisVisionTargeting(
      ()->-driver.getRawAxis(1),()->driver.getRawAxis(2),
      chassis, vision, navx)
    );

    shiftButton.whileHeld(new RunCommand(()->chassis.setGear(Gear.HIGH)));
    shiftButton.whenReleased(new RunCommand(()->chassis.setGear(Gear.LOW)));
    
    ejectBackButton.whileHeld(new PTMoveCargo(passthrough.kLowPower, -passthrough.kHighPower, passthrough));
    ejectBackButton.whileHeld(new FeederEjectCargo(feeder));
    ejectBackButton.whenPressed(()->passthrough.ptEnableColorSensors(false/*turn off front*/, false/*turn off back*/));

    ejectFrontButton.whileHeld(new PTMoveCargo(-passthrough.kHighPower, passthrough.kLowPower, passthrough));
    ejectFrontButton.whileHeld(new FeederEjectCargo(feeder));
    ejectFrontButton.whenPressed(()->passthrough.ptEnableColorSensors(false/*turn off front*/, false/*turn off back*/));
    
    shootButton.whileHeld(new FeederShootCargo(feeder));
    shootButton.whileHeld(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough));
    shootButton.whenPressed(()->passthrough.ptEnableColorSensors(true, true));
    
    shootButton.whenReleased(new InstantCommand(()->shooter.bottomMotor.set(0.0)));
    shootButton.whenReleased(new InstantCommand(()->shooter.topMotor.set(0.0)));
    
    spoolShooterButton.whileHeld(new ShooterSpoolUp(shooter));
  
    climbButtonManual.whileHeld(new RunCommand(()->climber.winchMotor.set(operator.getRawAxis(1))));
    climbButtonManual.whileHeld(new RunCommand(()->climber.hookMotor.set(operator.getRawAxis(2))));
    climbButtonManual.whenReleased(new InstantCommand(()->climber.hookMotor.set(0.0)));
    climbButtonManual.whenReleased(new InstantCommand(()->climber.winchMotor.set(0.0)));
    

    /****************************
     * Cargo Handling operations
     * *************************/
    //do smart cargo aware version
    intakeFrontButton.whileHeld(new ConditionalCommand(
      new IntakeDown(frontIntake), 
      new InstantCommand(()-> frontIntake.intakeOff()), 
      ()->passthrough.ptGetCargoLimit()));
    intakeFrontButton.whenPressed(new ConditionalCommand(
      new PTMoveCargo(passthrough.kHighPower,0.0,passthrough), 
      new InstantCommand(()->{}, passthrough), 
      ()->passthrough.ptGetCargoLimit()).withName("Intaking Front"));
    intakeFrontButton.whenPressed(()->passthrough.ptEnableColorSensors(true/*front sensor on*/, false /*back sensor off*/));
    intakeFrontButton.whenReleased(new InstantCommand(()->{}, passthrough));
   
    intakeBackButton.whileHeld(new ConditionalCommand(
      new IntakeDown(backIntake), 
      new InstantCommand(()-> backIntake.intakeOff()), 
      ()->passthrough.ptGetCargoLimit()));
    intakeBackButton.whenPressed(new ConditionalCommand(
      new PTMoveCargo(0,passthrough.kHighPower,passthrough), 
      new InstantCommand(()->{}, passthrough), 
      ()->passthrough.ptGetCargoLimit()));
    intakeBackButton.whenPressed(()->passthrough.ptEnableColorSensors(false/*front sensor off*/, true /*back sensor on*/));
    intakeBackButton.whenReleased(new InstantCommand(()->{}, passthrough));


    
    //Eject cargo out back intake
    ejectCargoOutBack = new Trigger(
      ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getOpposingColor();}
    );
    ejectCargoOutBack.whenActive(new ConditionalCommand(
      new PTMoveCargo(passthrough.kLowPower, -passthrough.kHighPower,passthrough).withTimeout(3).withName("EjectingCargoOutFront"), 
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled));
    
    //Eject cargo out front intake
    ejectCargoOutFront = new Trigger(
      ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getOpposingColor();}
    );
    ejectCargoOutFront.whenActive(new ConditionalCommand(
      new PTMoveCargo(-passthrough.kHighPower, passthrough.kLowPower, passthrough).withTimeout(3).withName("EjectingCargoOutBack"), 
      new InstantCommand(()->{}), 
      ()->passthrough.backSensorEnabled));

    //Load cargo from the front intake
    loadCargoFrontIntake = new Trigger(
      ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getTeamColor();}
    );
    loadCargoFrontIntake.whenActive(new ConditionalCommand(
      new PTLoadCargo(passthrough,feeder, true/*direction is front*/).withTimeout(2).withName("LoadingCargo"),
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled));
    
    //Load cargo from the back intake
    loadCargoFromBack = new Trigger(
      ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getTeamColor();}
    );
    loadCargoFromBack.whenActive(new ConditionalCommand(
      new PTLoadCargo(passthrough,feeder, false/*direction is back*/).withTimeout(2).withName("LoadingCargo"),
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled));
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
