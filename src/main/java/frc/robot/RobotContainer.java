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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisDriveArcade;
import frc.robot.commands.ChassisDriveToHeadingBasic;
import frc.robot.commands.ChassisPath;
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.FeederEjectCargo;
import frc.robot.commands.FeederFireOneShot;
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
  public CargoColorSensor cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kOnboard);
  // public CargoColorSensor cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
  

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
  
  Trigger ejectCargoOutBack;
  Trigger ejectCargoOutFront;
  
  Trigger loadCargoFrontIntake;
  Trigger loadCargoFromBack;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SmartDashboard.putData(new FeederFireOneShot(feeder));
    SmartDashboard.putData(new RunCommand(() -> shooter.setRPM(800)).withName("ShooterFeederTesting"));
    SmartDashboard.putNumber("compressor/amps",compressor.getCurrent());


    /**************************************************
     * Auto Stuff
     ************************************************/
    autoWaitTimer = SmartDashboard.getNumber("Auto Startup Delay", 0);
    SmartDashboard.putNumber("Auto Startup Delay",0);
    SmartDashboard.setPersistent("Auto Startup Delay");

    Command specialAuto = new InstantCommand(()->{})
    .andThen(new WaitCommand(autoWaitTimer))
    .andThen(new IntakeDown(backIntake)
      .alongWith(new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough))
      .alongWith(new ShooterSpoolUp(shooter))
      .alongWith(new ChassisPath(chassis, "Special Internal", true))
      ).withTimeout(4.0)
    .andThen(new InstantCommand(() -> {shooter.setRPM(2850);}))
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->25, 5, 5/12.0, navx, chassis).withTimeout(3.0))
    .andThen(
      new FeederShootCargo(feeder)
      .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
      .withTimeout(5)
      )
    .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
    .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
  ;
    //Center Auto
    Command centerAuto2Shot = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Center Internal", true))
        ).withTimeout(4.0)
      .andThen(new InstantCommand(() -> {shooter.setRPM(2100);}))
      .andThen(new ChassisDriveToHeadingBasic(0, ()->-30, 5, 5/12.0, navx, chassis).withTimeout(3.0))
      .andThen(
        new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(5)
        )
      .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;
    //Center Auto (4 Ball)
    Command centerAuto4Shot = new InstantCommand(()->{})
      .andThen(new WaitCommand(0)) // Should be autowait timer
      .andThen(new ParallelDeadlineGroup(
      new ChassisPath(chassis, "Center 4 Internal", true, Chassis.MaxAccelerationMetersPerSecondSquared, 2),
      new Command[] {
        new IntakeDown(backIntake),
        new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough),
        new InstantCommand(() -> {shooter.setRPM(2200);})
      }))
      .andThen(new WaitCommand(0.4))

      .andThen(
        new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(1.2)
        )
      .andThen(new ChassisPath(chassis, "Center 4", true))

      .andThen(new IntakeDown(backIntake)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(1.2)
      )

      .andThen(new ParallelDeadlineGroup(new ChassisPath(chassis, "Center 4 Return", false), 
        new Command[] {
          new IntakeDown(backIntake),
          new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough),
          new InstantCommand(() -> {shooter.setRPM(2300);})
        }))
      // .andThen(new ChassisPath(chassis, "Center 4 Return", false)
      //   .alongWith(new IntakeDown(backIntake).withTimeout(2.5))
      //   .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough).withTimeout(2.5))
      //   .alongWith(new ShooterSpoolUp(shooter).withTimeout(2.5))
      //   )

      .andThen(new InstantCommand(() -> {shooter.setRPM(2300);}))
      // .andThen(new ChassisDriveToHeadingBasic(0, ()->15, 5, 5/12.0, navx, chassis).withTimeout(1.5))
      .andThen(
        new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(1.2)
        )

      .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;
    //Right side auto
    Command rightAuto2Shot = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Right Internal", true))
        ).withTimeout(4.0)
      .andThen(new InstantCommand(() -> {shooter.setRPM(2300 + 50);}))
      .andThen(new ChassisDriveToHeadingBasic(0, ()->30, 5, 5/12.0, navx, chassis).withTimeout(3.0))
      .andThen(
        new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(5)
        )
      .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;
    //Right side auto (4 Ball)
    Command rightAuto4Shot = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Right Internal", true))
        ).withTimeout(4.0)
      .andThen(new InstantCommand(() -> {shooter.setRPM(2300);}))
      .andThen(new ChassisDriveToHeadingBasic(0, ()->30, 5, 5/12.0, navx, chassis).withTimeout(2.0))
      .andThen(new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(1.2)
      )
      .andThen(new ChassisPath(chassis, "Right 4", false)
        .alongWith(new IntakeDown(frontIntake))
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
      )

      .andThen(new InstantCommand(() -> {frontIntake.intakeOff();}, frontIntake, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;
    //Left side auto
    Command leftAuto2Shot = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new IntakeDown(backIntake)
        .alongWith(new PTMoveCargo(passthrough.kHighPower, passthrough.kHighPower, passthrough))
        .alongWith(new ShooterSpoolUp(shooter))
        .alongWith(new ChassisPath(chassis, "Left Internal", true))
        ).withTimeout(4.0)
      .andThen(new InstantCommand(() -> {shooter.setRPM(2500+50+75);})) // Bad battery while testing, plz fix
      .andThen(new ChassisDriveToHeadingBasic(0, ()->-25, 5, 5/12.0, navx, chassis).withTimeout(3.0))
      .andThen(
        new FeederShootCargo(feeder)
        .alongWith(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough))
        .withTimeout(5)
        )
      .andThen(new InstantCommand(() -> {backIntake.intakeOff();}, backIntake, chassis, passthrough, feeder, shooter))
      .andThen(new InstantCommand(() -> {shooter.setRPM(0);}))
    ;

    Command taxiAuto = new InstantCommand(()->{})
      .andThen(new WaitCommand(autoWaitTimer))
      .andThen(new ChassisPath(chassis, "Taxi", true))
    ;

    Command testAuto = new InstantCommand(()->{})
      .andThen(new ChassisPath(chassis, "Center 4 Return", false))
    ;

    autoChooser.setDefaultOption("Taxi", taxiAuto);
    autoChooser.addOption("Center 2 Ball", centerAuto2Shot);
    autoChooser.addOption("Center 4 Ball", centerAuto4Shot);
    autoChooser.addOption("Right 2 Ball", rightAuto2Shot);
    // autoChooser.addOption("Right 4 Ball", rightAuto4Shot);
    autoChooser.addOption("Left 2 Ball", leftAuto2Shot);
    autoChooser.addOption("Do nothing", new InstantCommand(()->{}));
    autoChooser.addOption("Special Auto", specialAuto); //who knows
    autoChooser.addOption("Test Auto", testAuto);
    


    //autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("autos/autoSelection", autoChooser);
    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    //Configure our autonomous commands, and make sure drive team can select what they want
    switch(Constants.botName){
    case COMP:
    cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kMXP);
    //  cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kOnboard);
    break;
    case PRACTICE:
     cargoColorSensorFront = new CargoColorSensor("front",I2C.Port.kOnboard , Rev2mDistanceSensor.Port.kMXP);
    //  cargoColorSensorBack = new CargoColorSensor("back",I2C.Port.kMXP, Rev2mDistanceSensor.Port.kOnboard);
    break;
    }

    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);


    /*************************************************
     * Default Commands
     *************************************************/
    
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
    
    ejectBackButton.whileHeld(new PTMoveCargo(-0.2, -1.0, passthrough));
    ejectBackButton.whileHeld(new FeederEjectCargo(feeder));
    ejectBackButton.whenPressed(()->passthrough.ptEnableColorSensors(false/*turn off front*/, false/*turn off back*/));

    ejectFrontButton.whileHeld(new PTMoveCargo(-1.0, -0.2, passthrough));
    ejectFrontButton.whileHeld(new FeederEjectCargo(feeder));
    ejectFrontButton.whenPressed(()->passthrough.ptEnableColorSensors(false/*turn off front*/, false/*turn off back*/));
    
    shootButton.whileHeld(new FeederShootCargo(feeder));
    shootButton.whileHeld(new PTMoveCargo(passthrough.kHighPower,passthrough.kHighPower,passthrough));
    shootButton.whenPressed(()->passthrough.ptEnableColorSensors(true, true));
    
    shootButton.whenReleased(new InstantCommand(()->shooter.bottomMotor.set(0.0)));
    shootButton.whenReleased(new InstantCommand(()->shooter.topMotor.set(0.0)));

    //Only works when driver is holding limelight targeting and has vision target
    spoolToLimeDistance.whileHeld(new RunCommand(()->shooter.setRPMForDistance(vision.getDistanceToUpperHub()),shooter));
    spoolToLimeDistance.whenReleased(new InstantCommand(()->shooter.setRPM(0),shooter));
    
    spoolShooterButton.whileHeld(new ShooterSpoolUp(shooter));
    
    climbButtonManual.whileHeld(new RunCommand(()->climber.winchMotor.set(-operator.getRawAxis(1))));
    // climbButtonManual.whileHeld(new RunCommand(()->climber.hookMotor.set(operator.getRawAxis(2))));
    // climbButtonManual.whenReleased(new InstantCommand(()->climber.hookMotor.set(0.0)));
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


    //TODO sensors busted, need fixing
    // Eject cargo out back intake
    // ejectCargoOutBack = new Trigger(
    //   ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getOpposingColor();}
    // );
    // ejectCargoOutBack.whenActive(new ConditionalCommand(
    //   new PTMoveCargo(passthrough.kLowPower, -passthrough.kHighPower,passthrough).withTimeout(3).withName("EjectingCargoOutFront"), 
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.frontSensorEnabled));

    // //Eject cargo out front intake
    // ejectCargoOutFront = new Trigger(
    //   ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getOpposingColor();}
    // );
    // ejectCargoOutFront.whenActive(new ConditionalCommand(
    //   new PTMoveCargo(-passthrough.kHighPower, passthrough.kLowPower, passthrough).withTimeout(3).withName("EjectingCargoOutBack"), 
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.backSensorEnabled));

    // //Load cargo from the front intake
    // loadCargoFrontIntake = new Trigger(
    //   ()->{return cargoColorSensorFront.getColor()==cargoColorSensorFront.getTeamColor();}
    // );
    // loadCargoFrontIntake.whenActive(new ConditionalCommand(
    //   new PTLoadCargo(passthrough,feeder, true/*direction is front*/).withTimeout(2).withName("LoadingCargo"),
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.frontSensorEnabled));
    
    // //Load cargo from the back intake
    // loadCargoFromBack = new Trigger(
    //   ()->{return cargoColorSensorBack.getColor()==cargoColorSensorBack.getTeamColor();}
    // );
    // loadCargoFromBack.whenActive(new ConditionalCommand(
    //   new PTLoadCargo(passthrough,feeder, false/*direction is back*/).withTimeout(2).withName("LoadingCargo"),
    //   new InstantCommand(()->{}), 
    //   ()->passthrough.frontSensorEnabled));
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
