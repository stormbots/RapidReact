// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    public CANSparkMax motorFeederFront = new CANSparkMax(11,MotorType.kBrushless);
    public CANSparkMax motorFeederBack = new CANSparkMax(12,MotorType.kBrushless);
    
    public RelativeEncoder encoderFeederFront = motorFeederFront.getEncoder();
    public RelativeEncoder encoderFeederBack = motorFeederBack.getEncoder();
    

    private double kFeederSpeed;
    
    boolean isCargoInFeeder = false;
    double previousDistance = 15;
    
    public Feeder() {

      switch(Constants.botName){
        case COMP:
         motorFeederFront.setInverted(true);
         motorFeederBack.setInverted(false);
        break;
        case PRACTICE:
         motorFeederFront.setInverted(false);
         motorFeederBack.setInverted(true);
        break;
        }
        
       

        motorFeederFront.setSmartCurrentLimit(30);
        motorFeederBack.setSmartCurrentLimit(30);

        kFeederSpeed = 0.7;

        encoderFeederFront.setPosition(0.0);
        encoderFeederBack.setPosition(0.0);

        encoderFeederFront.setPositionConversionFactor(6.0/10.0);
        encoderFeederBack.setPositionConversionFactor(6.0/10.0);//TODO get conversion factors for comp bot
    }

    public void feederRun(){
      motorFeederFront.set(kFeederSpeed);
      motorFeederBack.set(kFeederSpeed); 
    }

    public void feederEject(){
      motorFeederFront.set(-kFeederSpeed*.5);
      motorFeederBack.set(-kFeederSpeed);
    }

    public void feederOff(){
      motorFeederFront.set(0.0);
      motorFeederBack.set(0.0);
    }
    
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Feeder/Encoder Front", encoderFeederFront.getPosition());
      SmartDashboard.putNumber("Feeder/Encoder Back", encoderFeederBack.getPosition());
      SmartDashboard.putNumber("Feeder/front current", motorFeederFront.getOutputCurrent());
      SmartDashboard.putNumber("Feeder/back current", motorFeederBack.getOutputCurrent());
    }
    @Override
    public void simulationPeriodic() {
        // This method   will be called once per scheduler run during simulation
    }
}

