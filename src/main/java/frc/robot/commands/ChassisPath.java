package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Chassis;

public class ChassisPath extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(Chassis.sVolts, Chassis.vVoltSecondsPerMeter, Chassis.aVoltSecondsSquaredPerMeter),
    Chassis.DriveKinematics, 10);

  private TrajectoryConfig config = new TrajectoryConfig(Chassis.MaxSpeedMetersPerSecond, Chassis.MaxAccelerationMetersPerSecondSquared)
    .setKinematics(Chassis.DriveKinematics).addConstraint(autoVoltageConstraint);
    
  private Chassis chassis;
  private Command ramseteCommand;
  private String trajectoryName;
  private boolean reverse;

  public ChassisPath(Chassis c, String path, boolean reverse) {
    this.chassis = c;
    this.trajectoryName = path;
    this.reverse = reverse;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Trajectory trajectory = loadTrajectory(trajectoryName);
    
    chassis.reverse = this.reverse;
    chassis.resetOdometry(trajectory.getInitialPose());

    ramseteCommand = new RamseteCommand(trajectory, chassis::getPose,
        new RamseteController(Chassis.RamseteB, Chassis.RamseteZeta),
        new SimpleMotorFeedforward(Chassis.sVolts, Chassis.vVoltSecondsPerMeter, Chassis.aVoltSecondsSquaredPerMeter),
        Chassis.DriveKinematics,
        chassis::getWheelSpeeds,
        new PIDController(Chassis.PDriveVel, 0, 0),
        new PIDController(Chassis.PDriveVel, 0, 0),
        chassis::tankDriveVolts);
        /*NOTE would normally add chassis at end of list for requires, but this overseer command handles it*/
    ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(false);
    chassis.tankDriveVolts(0, 0);
    chassis.reverse = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ramseteCommand != null && ramseteCommand.isFinished()) {
      return true;
    }
    return false;
  }

  public Trajectory loadTrajectory(String name) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + name + ".wpilib.json");
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      switch (name) {
        case "Internal 1":
          return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
              //new Translation2d(0, 0)
            ),
            new Pose2d(-2, 0, new Rotation2d(0, 0)),
          config);
        case "Internal 2":
          return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
              //new Translation2d(0, 0)
            ),
            new Pose2d(-1.8, 0, new Rotation2d(0, 0)),
          config);
        default:
          DriverStation.reportError("Unable to open trajectory!", ex.getStackTrace());
          return null;
      }
    }
  }
}
