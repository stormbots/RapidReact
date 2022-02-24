
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;

/** An example command that uses an example subsystem. */
public class PTEjectCargoBack extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Passthrough passthrough;
  private final Boolean causedByTrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PTEjectCargoBack(Passthrough passthrough, Boolean causedByTrigger) {
    this.passthrough = passthrough;
    this.causedByTrigger = causedByTrigger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passthrough);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (causedByTrigger == false){
      // if done by manual button press reset cargo counter
      passthrough.ptResetCargo();
    }
    else{
      //if caused automatically do not reset cargo counter
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    passthrough.ptEjectBack();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.ptOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
