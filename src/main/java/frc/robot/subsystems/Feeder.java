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
    CANSparkMax motorFeederFront = new CANSparkMax(11,MotorType.kBrushless);
    CANSparkMax motorFeederBack = new CANSparkMax(12,MotorType.kBrushless);
    
    public RelativeEncoder encoderFeederFront = motorFeederFront.getEncoder();
    public RelativeEncoder encoderFeederBack = motorFeederBack.getEncoder();
    

    private double kFeederSpeed;
    
    boolean isCargoInFeeder = false;
    double previousDistance = 15;
    
    public Feeder() {

      switch(Constants.botName){
        case PRACTICE:
        break;
        case COMP:
        }
        
        motorFeederFront.setInverted(false);
        motorFeederBack.setInverted(true);

        motorFeederFront.setSmartCurrentLimit(30);
        motorFeederBack.setSmartCurrentLimit(30);

        kFeederSpeed = 0.6;

        encoderFeederFront.setPosition(0.0);
        encoderFeederBack.setPosition(0.0);
        encoderFeederFront.setPositionConversionFactor(6.0/10.0);
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
      SmartDashboard.getNumber("Feeder Encoder Front", encoderFeederFront.getPosition());
    }
    @Override
    public void simulationPeriodic() {
        // This method   will be called once per scheduler run during simulation
    }
}

