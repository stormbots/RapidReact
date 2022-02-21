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
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.FeederEjectCargo;
import frc.robot.commands.FeederShootCargo;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.PTEjectCargoBack;
import frc.robot.commands.PTEjectCargoFront;
import frc.robot.commands.PTIntakeCargo;
import frc.robot.commands.PTLoadCargo;
import frc.robot.commands.PTShootCargo;
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
  public CargoColorSensor cargoColorSensor = new CargoColorSensor(I2C.Port.kOnboard, Rev2mDistanceSensor.Port.kMXP);
  public Ultrasonic ptUltrasonic = new Ultrasonic(8,9); //TODO: Find actual configuration for this
  public Vision vision = new Vision(navx);
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  // 
  // SUBSYSTEMS
  //
  public Chassis chassis = new Chassis(navx);
  public Intake intake = new Intake(new CANSparkMax(7,MotorType.kBrushless));
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
  JoystickButton shiftButton = new JoystickButton(driver, 7);
  JoystickButton aimButton = new JoystickButton(driver, 8);

  public Joystick operator = new Joystick(1);
  JoystickButton ejectBackButton = new JoystickButton(operator, 6);
  JoystickButton ejectFrontButton = new JoystickButton(operator, 5);
  JoystickButton intakeButton = new JoystickButton(operator, 4);
  JoystickButton shootButton = new JoystickButton(operator, 1);
  JoystickButton spoolShooterButton = new JoystickButton(operator, 2);
  JoystickButton climbButton = new JoystickButton(operator, 7);
  JoystickButton climbButton2 = new JoystickButton(operator, 8);

  // Used to communicate auto commands to dashboard.
  SendableChooser<Command> autoChooser = new SendableChooser<>();


  private ChassisVisionTargeting chassisVisionTargeting = new ChassisVisionTargeting(()->0,()->0,chassis, vision, navx);
  
  Trigger ejectCargo;
  Trigger loadCargo;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Configure our autonomous commands, and make sure drive team can select what they want

    autoChooser.setDefaultOption("Does nothing", new InstantCommand(()->{}));
    autoChooser.addOption("Also nothing", new InstantCommand(()->{}));
    SmartDashboard.putData("autos/autoSelection", autoChooser);
    SmartDashboard.putData("ChassisVisionTargeting", chassisVisionTargeting);



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
      chassis, vision, navx));

    shiftButton.whileHeld(new RunCommand(()->chassis.setGear(Gear.HIGH)));
    shiftButton.whenReleased(new RunCommand(()->chassis.setGear(Gear.LOW)));
    
    ejectBackButton.whileHeld(new PTEjectCargoBack(passthrough));
    ejectBackButton.whileHeld(new FeederEjectCargo(feeder));

    ejectFrontButton.whileHeld(new PTEjectCargoFront(passthrough));
    ejectFrontButton.whileHeld(new FeederEjectCargo(feeder));
    
    
    // shootButton.whileHeld(new RunCommand(()->shooter.topMotor.set(0.2)));
    // shootButton.whileHeld(new RunCommand(()->shooter.bottomMotor.set(0.2*.9)));
    shootButton.whileHeld(new FeederShootCargo(feeder));
    shootButton.whileHeld(new PTLoadCargo(passthrough));
    
    shootButton.whenReleased(new RunCommand(()->shooter.bottomMotor.set(0.0)));
    shootButton.whenReleased(new RunCommand(()->shooter.bottomMotor.set(0.0)));
    
    spoolShooterButton.whileHeld(new ShooterSpoolUp(shooter));
  
    //do smart cargo aware version
    intakeButton.whileHeld(new ConditionalCommand(
      new IntakeDown(intake), 
      new InstantCommand(()-> intake.intakeOff()), 
      ()->passthrough.ptGetCargoLimit()));
    intakeButton.whenPressed(new ConditionalCommand(
      new PTIntakeCargo(passthrough), 
      new InstantCommand(()->{}, passthrough), 
      ()->passthrough.ptGetCargoLimit()));
    intakeButton.whenReleased(new InstantCommand(()->{}, passthrough));

    climbButton.whileHeld(new RunCommand(()->climberTestMotor.set(-0.1)));
    climbButton.whenReleased(new InstantCommand(()->climberTestMotor.set(0)));
    climbButton2.whileHeld(new RunCommand(()->climberTestMotor.set(0.1)));
    climbButton2.whenReleased(new InstantCommand(()->climberTestMotor.set(0)));

   
    ejectCargo = new Trigger(
      ()->{return cargoColorSensor.getColor()==cargoColorSensor.getOpposingColor()/*TODO this needs to be changed to teamcolor*/;}
    );
    ejectCargo.whenActive(new PTEjectCargoBack(passthrough).withTimeout(2));//TODO needs to be tuned

    loadCargo = new Trigger(
      ()->{return cargoColorSensor.getColor()==cargoColorSensor.getTeamColor()/*TODO this needs to be changed to  !teamcolor*/;}
    );
    loadCargo.whenActive(new PTLoadCargo(passthrough,feeder).withTimeout(2)); 
    
    // Trigger moveCargoToFeeder = new Trigger(
    //   ()->{return passthrough.ptCargoInPT() == true;}
    // );
    // moveCargoToFeeder.whileActiveContinuous(new PTLoadCargo(passthrough));
    
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
