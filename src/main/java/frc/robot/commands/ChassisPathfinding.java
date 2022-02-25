package frc.robot.commands;

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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class ChassisPathfinding extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Chassis chassis;
    
    private DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.sVolts, Constants.vVoltSecondsPerMeter, Constants.aVoltSecondsSquaredPerMeter),
      Constants.DriveKinematics, 10);
  
    private TrajectoryConfig config = new TrajectoryConfig(Constants.MaxSpeedMetersPerSecond, Constants.MaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.DriveKinematics).addConstraint(autoVoltageConstraint);
  
    private RamseteCommand ramseteCommand;
  
    public ChassisPathfinding(Chassis c) {
      chassis = c;
      addRequirements(chassis);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(2, 0),
          new Translation2d(2, -2)
        ),
        new Pose2d(4, -2, new Rotation2d(1, 0)),
      config);
      
      chassis.resetOdometry(trajectory.getInitialPose());
  
      ramseteCommand = new RamseteCommand(trajectory, chassis::getPose,
          new RamseteController(Constants.RamseteB, Constants.RamseteZeta),
          new SimpleMotorFeedforward(Constants.sVolts, Constants.vVoltSecondsPerMeter, Constants.aVoltSecondsSquaredPerMeter),
          Constants.DriveKinematics,
          chassis::getWheelSpeeds,
          new PIDController(Constants.PDriveVel, 0, 0),
          new PIDController(Constants.PDriveVel, 0, 0),
          chassis::tankDriveVolts,
          chassis);
      
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
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
