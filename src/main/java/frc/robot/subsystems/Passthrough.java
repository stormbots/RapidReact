
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Passthrough extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    CANSparkMax motorPTFront = new CANSparkMax(9,MotorType.kBrushless);
    CANSparkMax motorPTBack = new CANSparkMax(10,MotorType.kBrushless);
    
    public Ultrasonic passthroughUltrasonic = new Ultrasonic(1, 2);
    
    private double kPTSpeed;
    private boolean isCargoInPT;
    
    public Passthrough() {
        switch(Constants.botName){
        case PRACTICE:
        
        //Set Inversions TODO Tune Inversions
        motorPTFront.setInverted(true);
        motorPTBack.setInverted(false);

        //Set Current Limits TODO Currently arbitrary Increase/Decrease as needed
        motorPTFront.setSmartCurrentLimit(30);
        motorPTBack.setSmartCurrentLimit(30);
        
        Ultrasonic.setAutomaticMode(true);
        isCargoInPT = true;
        break;
        case COMP:
        }
    
        
    }

    
    public void ptRun(){
      motorPTFront.set(kPTSpeed);
      motorPTBack.set(kPTSpeed);
    }
    
    public void ptOff(){
      motorPTFront.set(0.0);
      motorPTBack.set(0.0);
    }

    public void ptEject(){
      motorPTFront.set(kPTSpeed);
      motorPTBack.set(-kPTSpeed);
    }

    public Boolean getUltrasonicRange(){
      if(passthroughUltrasonic.getRangeInches() < 12){
        return isCargoInPT;
      }
      else{
        return !isCargoInPT;
      }
    }

    
    @Override
    public void periodic() {
      SmartDashboard.getNumber("Ultrasonic Range", passthroughUltrasonic.getRangeInches());
    }
    @Override
    public void simulationPeriodic() {
        // This method   will be called once per scheduler run during simulation
    }
}

