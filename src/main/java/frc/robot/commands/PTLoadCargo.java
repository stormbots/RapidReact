
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Passthrough;

/** Place Cargo where it should be. */
public class PTLoadCargo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Passthrough passthrough;
  private final Feeder feeder;
  private double passthroughTargetDistance=0;
  private double feederTargetDistance=0;
  private Boolean direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PTLoadCargo(Passthrough passthrough, Feeder feeder, Boolean direction) {
    this.passthrough = passthrough;
    this.feeder = feeder;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passthrough);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    passthrough.encoderPTFront.setPosition(0);
    feeder.encoderFeederFront.setPosition(0);
    passthrough.encoderPTBack.setPosition(0);
    feeder.encoderFeederBack.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(passthrough.ptGetNumberOfCargo()==0){
      passthroughTargetDistance = 30;//TODO get comp bot distances
      feederTargetDistance = 30;
      if (direction = false/*if direction is back*/){
        feeder.feederRun();
        passthrough.setPTpower(0, 1);//TODO determine if this works at 0 not 0.1
      }
      else if(direction = true/*if direction is front*/){
        feeder.feederRun();
        passthrough.setPTpower(1, 0);
      }
      SmartDashboard.putString("passthrough/PTLOADINGCARGO", "Loading 0");
    }
    else if (passthrough.ptGetNumberOfCargo() == 1){
        //put cargo in bottom spot 
      // passthrough.setDistanceBottom();
      passthroughTargetDistance = 18;
      if (direction = false/*if direction is back*/){
        passthrough.setPTpower(0, 1);//TODO determine if this works at 0 not 0.1
      }
      else if(direction = true/*if direction is front*/){
        passthrough.setPTpower(1, 0);
      }
      SmartDashboard.putString("passthrough/PTLOADINGCARGO", "Loading 1");
    }
    else {
      //too many cargo, what?
    }

    //shut off feeder if it moved it's distance
    if (feeder.encoderFeederFront.getPosition()>feederTargetDistance
    || feeder.encoderFeederBack.getPosition()>feederTargetDistance){
      feeder.feederOff();
      SmartDashboard.putString("passthrough/FEEDERTURINGOFF", "OFF");
    }
    else{
      SmartDashboard.putString("passthrough/FEEDERTURINGOFF", "ON");
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.setPTpower(0,0);
    feeder.feederOff();
    passthrough.ptIncrementCargo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check encoders and stop it properly
    if(passthrough.encoderPTFront.getPosition() > passthroughTargetDistance 
    || passthrough.encoderPTBack.getPosition() > passthroughTargetDistance){
      return true;
    }
    return false;
  }
}
