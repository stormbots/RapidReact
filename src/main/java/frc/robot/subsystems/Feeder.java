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
    
    RelativeEncoder encoderFeederFront = motorFeederFront.getEncoder();
    RelativeEncoder encoderFeederBack = motorFeederBack.getEncoder();
    

    private double kFeederSpeed;
    
    boolean isCargoInFeeder = false;
    double previousDistance = 15;
    
    public Feeder() {
        switch(Constants.botName){
        case PRACTICE:
        
        //Set Inversions TODO Tune Inversions
        motorFeederFront.setInverted(true);
        motorFeederBack.setInverted(false);

        //Set Current Limits TODO Currently arbitrary Increase/Decrease as needed
        motorFeederFront.setSmartCurrentLimit(30);
        motorFeederBack.setSmartCurrentLimit(30);

        kFeederSpeed = 0.1;
        
        
        encoderFeederFront.setPosition(0.0);
        encoderFeederBack.setPosition(0.0);
        
        break;
        case COMP:
        }
        
    }

    public void feederIntake(){
        motorFeederFront.set(kFeederSpeed);
        motorFeederBack.set(kFeederSpeed);
       
    }

    public void getFeederEncoders(){
      if (encoderFeederFront.getPosition() > 100){
        motorFeederFront.set(0.0);
        motorFeederBack.set(0.0);
        encoderFeederFront.setPosition(0.0);
        encoderFeederBack.setPosition(0.0);
      }
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

