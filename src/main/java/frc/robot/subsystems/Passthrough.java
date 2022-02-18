
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Passthrough extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    CANSparkMax motorPTFront = new CANSparkMax(9,MotorType.kBrushless);
    CANSparkMax motorPTBack = new CANSparkMax(10,MotorType.kBrushless);
    
    public Ultrasonic passthroughUltrasonic = new Ultrasonic(1, 2);
    
    double kPTSpeed;
    double kEjectDifference;
    boolean isCargoInPT;
    boolean isCargoInFeeder;
    double kFeederHeight;
    double kUltrasonicMaximumHeight;
  
    public Passthrough() {
        switch(Constants.botName){
        case PRACTICE:
        
        break;
        case COMP:
        }

        // SmartDashboard.putString("passthrough/faults",motorPTFront.getFaults());
        // SmartDashboard motorPTBack.getFaults();

        motorPTFront.setInverted(false);
        motorPTBack.setInverted(true);

        //Set Current Limits TODO Currently arbitrary Increase/Decrease as needed
        motorPTFront.setSmartCurrentLimit(30);
        motorPTBack.setSmartCurrentLimit(30);
        motorPTFront.setOpenLoopRampRate(0.2);
        motorPTBack.setOpenLoopRampRate(0.2);
        
        Ultrasonic.setAutomaticMode(true);
        isCargoInPT = false;
        isCargoInFeeder = false;
        kPTSpeed = 0.6;
        kFeederHeight = 12; //TODO get this from testing
        kUltrasonicMaximumHeight = 50;//TODO get this from testing 
        kEjectDifference = .8;
    }

    
    public void ptRun(){
      motorPTFront.set(kPTSpeed);
      motorPTBack.set(kPTSpeed);
    }
    
    public void ptOff(){
      motorPTFront.set(0.0);
      motorPTBack.set(0.0);
    }

    public void ptEjectBack(){
      motorPTFront.set(kPTSpeed*kEjectDifference);
      motorPTBack.set(-kPTSpeed);
    }

    public void ptEjectFront(){
      motorPTFront.set(-kPTSpeed);
      motorPTBack.set(kPTSpeed*kEjectDifference);
    }

    public Boolean ptCargoInPT(){
      if(passthroughUltrasonic.getRangeInches() < kFeederHeight){
        isCargoInPT = true;
        return isCargoInPT;
      }
      else{
        isCargoInPT = false;
        return isCargoInPT;
      }
    }

    public Boolean ptCargoInFeeder(){
      if(passthroughUltrasonic.getRangeInches() > kFeederHeight & passthroughUltrasonic.getRangeInches() < kUltrasonicMaximumHeight){
        isCargoInFeeder = true;
        return isCargoInFeeder;
      }
      else{
        isCargoInFeeder = false;
        return isCargoInFeeder;
      }
    }
    
 

    
    @Override
    public void periodic() {
      SmartDashboard.getNumber("Ultrasonic Range", passthroughUltrasonic.getRangeInches());
      Command current = getCurrentCommand();
      if(current !=null){
        SmartDashboard.putString("passthrough/command", current.getName());      }
    }
    @Override
    public void simulationPeriodic() {
        // This method   will be called once per scheduler run during simulation
    }
}

