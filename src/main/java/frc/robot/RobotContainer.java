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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisDriveArcade;
import frc.robot.commands.ChassisDriveToHeadingBasic;
import frc.robot.commands.ChassisPath;
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.FeederEjectCargo;
import frc.robot.commands.FeederRun;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.PTLoadCargo;
import frc.robot.commands.PassthroughRun;
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
  public CargoColorSensor cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kMXP);
  public CargoColorSensor cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
  

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
  JoystickButton shiftButton = new JoystickButton(driver, 5);
  JoystickButton aimButton = new JoystickButton(driver, 6);
  JoystickButton slowMode = new JoystickButton(driver, 8);

  public Joystick operator = new Joystick(1);
  JoystickButton ejectBackButton = new JoystickButton(operator, 6);
  JoystickButton ejectFrontButton = new JoystickButton(operator, 5);
  JoystickButton intakeFrontButton = new JoystickButton(operator, 4);
  JoystickButton intakeBackButton = new JoystickButton(operator, 2);
  JoystickButton shootButton = new JoystickButton(operator, 1);
  JoystickButton spoolShooterButton = new JoystickButton(operator, 7);
  JoystickButton spoolToLimeDistance = new JoystickButton(operator, 8);
  JoystickButton climbButtonManual = new JoystickButton(operator, 9);
  

  // Used to communicate auto commands to dashboard.
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  double autoWaitTimer=0;


  private ChassisVisionTargeting chassisVisionTargeting = new ChassisVisionTargeting(()->0,()->0,chassis, vision, navx);
  
  Trigger ejectCargoFrontSensor;
  Trigger ejectCargoBackSensor;
  
  Trigger loadCargoFrontSensor;
  Trigger loadCargoBackSensor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /**************************************************
     * Auto Stuff
     ************************************************/
    autoWaitTimer= SmartDashboard.getNumber("Auto Startup Delay", 0);
    SmartDashboard.putNumber("Auto Startup Delay",0);
    SmartDashboard.setPersistent("Auto Startup Delay");

    Command testAuto = new InstantCommand(()->{})
    .andThen(new WaitCommand(autoWaitTimer))
    .andThen(new IntakeDown(backIntake)
      .alongWith(new PassthroughRun(passthrough.kHighPower, passthrough.kHighPower, passthrough))
      .alongWith(new ShooterSpoolUp(shooter))
      .alongWith(new ChassisPath(chassis, "Internal 3", true))
      ).withTimeout(4.0)
    .andThen(new InstantCommand(() -> {shooter.setRPM(2850);}))
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->25, 5, 5/12.0, navx, chassis).withTimeout(3.0))
    .andThen(
      new FeederRun(feeder)
      .alongWith(new PassthroughRun(passthrough.kHighPower,passthrough.kHighPower,passthrough))
      .withTimeout(5)
      )
    .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
    .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
  ;
  ;
    //Center Auto
    Command leftAuto = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PassthroughRun(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Internal 1", true))
        ).withTimeout(4.0)
      //.andThen(new PTLoadCargo(passthrough, feeder, true))
      .andThen(new ChassisDriveToHeadingBasic(0, ()->-30, 5, 5/12.0, navx, chassis)
        .alongWith(new InstantCommand(()->{shooter.setRPM(2950);}))
        .withTimeout(3.0)
        )
      .andThen(new FeederRun(feeder)) //shoves cargo into feeder
      .andThen(new InstantCommand(() -> {backIntake.intakeOff();shooter.setRPM(0);}, backIntake, chassis, passthrough, feeder, shooter))
    ;
    //Right side auto
    Command rightAuto = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PassthroughRun(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Internal 2", true))
        ).withTimeout(4.0)
      .andThen(new InstantCommand(() -> {shooter.setRPM(2850 - 50);}))
      .andThen(new ChassisDriveToHeadingBasic(0, ()->25, 5, 5/12.0, navx, chassis).withTimeout(3.0))
      .andThen(
        new FeederRun(feeder)
        .alongWith(new PassthroughRun(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(5)
        )
      .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;

    Command taxiAuto = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new ChassisPath(chassis, "Internal 2", true))
    ;

    autoChooser.setDefaultOption("Taxi", taxiAuto);
    autoChooser.addOption("Center 1 Ball", leftAuto); //Center position
    autoChooser.addOption("Far Side 1 Ball", rightAuto); //side near guardrail
    autoChooser.addOption("Taxi", taxiAuto);
    autoChooser.addOption("Do nothing", new InstantCommand(()->{}));
    autoChooser.addOption("Special Auto", testAuto); //who knows


    //autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("autos/autoSelection", autoChooser);
    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    //Configure our autonomous commands, and make sure drive team can select what they want
    // switch(Constants.botName){
    // case COMP:
    //  cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kMXP);
    //  cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
    // break;
    // case PRACTICE:
    //  cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kOnboard , Rev2mDistanceSensor.Port.kMXP);
    //  cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kOnboard);
    // break;
    // }

    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    /**************************************************
     * Default Commands
     ************************************************/
    
    chassis.setDefaultCommand(
      // this one's really basic, but needed to get systems moving right away.
      new RunCommand(
        ()->{chassis.arcadeDrive(-driver.getRawAxis(1),driver.getRawAxis(2));}
        ,chassis)
      );
      slowMode.whileHeld(new RunCommand(
        ()->{chassis.arcadeDrive(-driver.getRawAxis(1) * .5, driver.getRawAxis(2) * .5);}
        ,chassis)
      );

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
    
    ejectBackButton.whileHeld(new PassthroughRun(passthrough.kLowPower, -passthrough.kHighPower, passthrough));
    ejectBackButton.whileHeld(new FeederEjectCargo(feeder));
    ejectBackButton.whenPressed(()->passthrough.ptEnableColorSensors(false, false));//turn off both sensor for eject

    ejectFrontButton.whileHeld(new PassthroughRun(-passthrough.kHighPower, passthrough.kLowPower, passthrough));
    ejectFrontButton.whileHeld(new FeederEjectCargo(feeder));
    ejectFrontButton.whenPressed(()->passthrough.ptEnableColorSensors(false, false));//turn off both sensor for eject
    
    shootButton.whileHeld(new FeederRun(feeder));
    shootButton.whileHeld(new PassthroughRun(passthrough.kHighPower,passthrough.kHighPower,passthrough));
    shootButton.whenPressed(()->passthrough.ptResetCargo());//not functional
    shootButton.whenPressed(()->passthrough.ptEnableColorSensors(true, true));//enables both sensors for testing

    shootButton.whenReleased(new InstantCommand(()->shooter.bottomMotor.set(0.0)));
    shootButton.whenReleased(new InstantCommand(()->shooter.topMotor.set(0.0)));

    //Only works when driver is holding limelight targeting and has vision target
    spoolToLimeDistance.whileHeld(new RunCommand(()->shooter.setRPMForDistance(vision.getDistanceToUpperHub()),shooter));
    spoolToLimeDistance.whenReleased(new InstantCommand(()->shooter.setRPM(0),shooter));
    
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
    intakeFrontButton.whileHeld(new ConditionalCommand(
      new PassthroughRun(passthrough.kHighPower,0.0,passthrough), 
      new InstantCommand(()->{}), 
      ()->passthrough.ptGetCargoLimit() && passthrough.getCurrentCommand()==null)
        .withName("Intaking Front")
      );
    intakeFrontButton.whenPressed(()->passthrough.ptEnableColorSensors(true, false));//front sensor on, back sensor off
    intakeFrontButton.whenReleased(new InstantCommand(()->{}, passthrough));
   
    // intakeBackButton.whileHeld(new ConditionalCommand(
    //   new IntakeDown(backIntake), 
    //   new InstantCommand(()-> backIntake.intakeOff()), 
    //   ()->passthrough.ptGetCargoLimit()));
    // intakeBackButton.whenPressed(new ConditionalCommand(
    //   new PassthroughRun(0,passthrough.kHighPower,passthrough), 
    //   new InstantCommand(()->{}, passthrough), 
    //   ()->passthrough.ptGetCargoLimit()));
    // intakeBackButton.whenPressed(()->passthrough.ptEnableColorSensors(false, true));//front sensor off, back sensor on
    // intakeBackButton.whenReleased(new InstantCommand(()->{}, passthrough));


    //TODO sensors busted, need fixing
    //Eject cargo out back intake
    ejectCargoFrontSensor = new Trigger(
      ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getOpposingColor();}
    );
    ejectCargoFrontSensor.whenActive(new ConditionalCommand(
      new PassthroughRun(passthrough.kLowPower, -passthrough.kHighPower,passthrough).withTimeout(.6).withName("EjectingCargoOutFront"), 
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled).withName("EjectingFrontSensor"));

    SmartDashboard.putData(new ConditionalCommand(
      new PassthroughRun(passthrough.kLowPower, -passthrough.kHighPower,passthrough).withTimeout(.6).withName("EjectingCargoOutFront"), 
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled).withName("EjectingFrontSensor"));
    
    //Eject cargo out front intake
    // ejectCargoBackSensor = new Trigger(
    //   ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getOpposingColor();}
    // );
    // ejectCargoBackSensor.whenActive(new ConditionalCommand(
    //   new PTMoveCargo(-passthrough.kHighPower, passthrough.kLowPower, passthrough).withTimeout(3).withName("EjectingCargoOutBack"), 
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.backSensorEnabled).withName("EjectingBackSensor"));

    //Load cargo from the front intake
    loadCargoFrontSensor = new Trigger(
      ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getTeamColor();}
    );
    loadCargoFrontSensor.whenActive(new ConditionalCommand(
      new PTLoadCargo(passthrough,feeder, true/*direction is front*/).withTimeout(.6).withName("LoadingCargo"),
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled).withName("LoadingFrontSensor"));
    SmartDashboard.putData(new ConditionalCommand(
      new PTLoadCargo(passthrough,feeder, true/*direction is front*/).withTimeout(.6).withName("LoadingCargo"),
      new InstantCommand(()->{}), 
      ()->passthrough.frontSensorEnabled).withName("LoadingFrontSensor"));
    // //Load cargo from the back intake
    // loadCargoBackSensor = new Trigger(
    //   ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getTeamColor();}
    // );
    // loadCargoBackSensor.whenActive(new ConditionalCommand(
    //   new PTLoadCargo(passthrough,feeder, false/*direction is back*/).withTimeout(2).withName("LoadingCargo"),
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.frontSensorEnabled).withName("LoadingBackSensor"));
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
