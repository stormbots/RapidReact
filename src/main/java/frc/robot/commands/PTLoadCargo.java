
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;

/** Place Cargo where it should be. */
public class PTLoadCargo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Passthrough passthrough;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PTLoadCargo(Passthrough passthrough) {
    this.passthrough = passthrough;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passthrough);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (passthrough.ptGetNumberOfCargo() == 1){
      //put cargo in bottom spot 
      passthrough.ptRun();
    }
    else {
      //put cargo in top spot
      passthrough.ptRun();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.ptOff();
    passthrough.ptIncrementCargo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
