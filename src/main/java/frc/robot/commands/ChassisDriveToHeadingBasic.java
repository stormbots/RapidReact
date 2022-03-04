/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.Clamp;
import com.stormbots.closedloop.FB;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveToHeadingBasic extends CommandBase {
  private final Chassis chassis;
  private AHRS gyro;
  private double targetDistance;
  private DoubleSupplier targetBearingSupplier;
  private double targetBearing;
  private double initialBearing;

  private double angleTolerance;
  private double distanceTolerance;

  
  SlewRateLimiter speedslew;

  MiniPID jankysolution = new MiniPID(0.2/30, 0, 0.0);

  /**
   * Creates a new ChassisDriveManual.
   * All distance units should be in Meters
   */
  public ChassisDriveToHeadingBasic(double targetDistance, DoubleSupplier targetBearingSupplier, double angleTolerance, double distanceTolerance, AHRS gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);

    this.chassis = chassis;
    this.gyro = gyro;
    this.targetDistance = targetDistance;
    this.targetBearingSupplier = targetBearingSupplier;

    this.angleTolerance = angleTolerance;
    this.distanceTolerance = distanceTolerance;


    speedslew = new SlewRateLimiter(0.5, 0); // 0.5 -> ACCEL DISTANCE

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    jankysolution.reset();


    chassis.leftEncoder.setPosition(0);
    chassis.rightEncoder.setPosition(0);
    
    
    targetBearing = targetBearingSupplier.getAsDouble();


    initialBearing = gyro.getAngle();
    jankysolution.setSetpoint(initialBearing + targetBearing);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // SmartDashboard.putNumber("Chassis/targetBearing", targetBearing);
    // SmartDashboard.putNumber("Chassis/targetBearingSupplier", targetBearingSupplier.getAsDouble());


    double currentAngle =  gyro.getAngle();
    //Uses PID to create a motion controlled turn value
    double turn = jankysolution.getOutput(currentAngle);
    
    double distance = chassis.getAverageDistance();

    double targetDistance = speedslew.calculate(this.targetDistance);

    double forwardSpeed = FB.fb(targetDistance, distance, 0.5); // TABI 0.4 || PRACTICE 0.4

    // if(Math.abs(targetBearing - currentAngle) > 20) {
    //   forwardSpeed = 0;
    // }

    // forwardSpeed = 0; //DEBUG

    turn+= Math.signum(turn)*0.08; //TODO: fIXME WHEN MINIPID WORKS PROPERLY
    // turn+= Math.signum(turn)*0.02; //TODO: fIXME WHEN MINIPID WORKS PROPERLY // TABIIIIIIIIIIIIIIIIIIIII


    chassis.arcadeDrive(
        forwardSpeed, 
        turn
      );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.arcadeDrive(0,0);
    // System.out.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nThat's all folks!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // angleTolerance = Math.copySign(angleTolerance, targetBearing.getAsDouble());

    // SmartDashboard.putNumber("Chassis/angle Error", gyro.getAngle() - (initialBearing + targetBearing) );
    // SmartDashboard.putNumber("Chassis/position Error", chassis.getAverageDistance() - targetDistance);
    SmartDashboard.putBoolean("Chassis/Exit Total", ( Clamp.bounded(gyro.getAngle(), initialBearing+targetBearing-angleTolerance, initialBearing+targetBearing+angleTolerance)
    && ( Clamp.bounded(chassis.getAverageDistance(), targetDistance-distanceTolerance, targetDistance+distanceTolerance) ) ));

    // SmartDashboard.putBoolean("Chassis/Exit Angle", Clamp.bounded(gyro.getAngle(), initialBearing+targetBearing-angleTolerance, initialBearing+targetBearing+angleTolerance));

    return ( Clamp.bounded(gyro.getAngle(), initialBearing+targetBearing-angleTolerance, initialBearing+targetBearing+angleTolerance)
    && ( Clamp.bounded(chassis.getAverageDistance(), targetDistance-distanceTolerance, targetDistance+distanceTolerance) ) );






  }
}