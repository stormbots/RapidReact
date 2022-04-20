// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.stormbots.devices.BlinkenPattern;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  private double delay = Timer.getFPGATimestamp();
  private double matchTime;
  private boolean defualt = true;

  Spark ledModule = new Spark(0);

  public LED() {
    SmartDashboard.putNumber("LED/Debugging Value", 0.69);
    SmartDashboard.putBoolean("LED/Debugging", false);

    switch(DriverStation.getAlliance()){
      case Blue: 
      ledModule.set(.83);
      break;
      case Red:
      ledModule.set(.61);
      break;
      default:
    }
  }

  public void setGreen(){
    defualt = false;
    ledModule.set(0.77);
  }

  public void reset(){
    defualt = true;
  }

  public void setRainbow(){
    defualt = false;
  }

  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("LED/Debugging", false)){
      ledModule.set(SmartDashboard.getNumber("LED/Debugging Value", 0.69));
    }
    else if(defualt){
      matchTime = Timer.getMatchTime();
      SmartDashboard.putNumber("MatchTime", matchTime);

      if(matchTime <= 30.0 && matchTime > 15.0 && matchTime != -1 && DriverStation.isTeleop()){
        if (delay <= Timer.getFPGATimestamp()){
          delay = Timer.getFPGATimestamp() + 1;
          if(ledModule.get() >= 0.9){
            ledModule.set(.69);
          }
          else{
            ledModule.set(0.99);
          }
        }
      } 
      else if(matchTime <= 15.0 && matchTime != -1 && DriverStation.isTeleop()){
          if (delay <= Timer.getFPGATimestamp()){
            delay = Timer.getFPGATimestamp() + 0.4;
            if(ledModule.get() >= 0.9){
              ledModule.set(.63);
            }
            else{
              ledModule.set(0.99);
            }
          }
      } else {
        switch(DriverStation.getAlliance()){
          case Blue: 
          ledModule.set(.83);
          break;
          case Red:
          ledModule.set(.61);
          break;
          default:
          }
      }
    }

  }
}

