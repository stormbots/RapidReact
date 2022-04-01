
package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class FeederShootCargo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Feeder feeder;
  private Shooter shooter;
  double feederDelayedStopAt;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeederShootCargo(Feeder feeder) {
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }
  public FeederShootCargo(Feeder feeder,Shooter shooter) {
    this.feeder = feeder;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setIdleMode(IdleMode.kBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter==null){ //legacy mode
      feeder.feederRun();
      return;
    }


    //fancy new code
    if(shooter.isOnTargetRPM(50.0)){
      feederDelayedStopAt = Timer.getFPGATimestamp() + 0.03;
    }

    if(feederDelayedStopAt<Timer.getFPGATimestamp()){
      feeder.feederOff();
    }
    else{
      feeder.feederRun();
    }


    //if rpm drops low
    //set a wait timer
    //when wait timer = 0 / equal something
    //stop motors
    //wait until rpm is at our target again

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.feederOff();
    feeder.setIdleMode(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
