
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.closedloop.MiniPID;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CargoColorSensor.CargoColor;

public class Passthrough extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    CANSparkMax motorPTFront = new CANSparkMax(9,MotorType.kBrushless);
    CANSparkMax motorPTBack = new CANSparkMax(10,MotorType.kBrushless);
    
    public RelativeEncoder encoderPTFront;
    public RelativeEncoder encoderPTBack; 
    
    MiniPID ptPidFront;

    public Ultrasonic passthroughUltrasonic = new Ultrasonic(1, 2);
    
    public double kHighPower=0.8;
    public double kLowPower=0.6;
    double kPTSpeed;
    double kEjectDifference;
    double kFeederHeight;
    double kUltrasonicMaximumHeight;
    double numberOfCargo;
    double kDistanceToTop;
    double kDistanceToBottom;
    
   
    public Passthrough() {
        switch(Constants.botName){
        case PRACTICE:
        
        break;
        case COMP:
        }
        motorPTFront.setInverted(false);
        motorPTBack.setInverted(true);

        encoderPTBack=motorPTBack.getEncoder();
        encoderPTFront=motorPTFront.getEncoder();
        
        encoderPTBack.setPosition(0.0);
        encoderPTFront.setPosition(0.0);

        encoderPTFront.setPositionConversionFactor(6.0/10.0);

        motorPTFront.setSmartCurrentLimit(30);
        motorPTBack.setSmartCurrentLimit(30);
        motorPTFront.setOpenLoopRampRate(0.2);
        motorPTBack.setOpenLoopRampRate(0.2);
        
        Ultrasonic.setAutomaticMode(true);

        kPTSpeed = .8;
        // kFeederHeight = 12; //TODO get this from testing
        // kUltrasonicMaximumHeight = 50;//TODO get this from testing 
        kEjectDifference = 1.2;
        numberOfCargo = 0;
    }

    public void setPTpower(double front, double rear){
      motorPTFront.set(kPTSpeed);
      motorPTBack.set(kPTSpeed);
    }
    public void ptIncrementCargo(){
      numberOfCargo += 1;
    }

    public void ptResetCargo(){
      numberOfCargo = 0;
    }

    public double ptGetNumberOfCargo(){
      return numberOfCargo;
    }

    public Boolean ptGetCargoLimit(){
      if (numberOfCargo <2){
        return true;
      }
      return false;
    }
    
    @Override
    public void periodic() {
      SmartDashboard.getNumber("Ultrasonic Range", passthroughUltrasonic.getRangeInches());
      Command current = getCurrentCommand();
      if(current !=null){
        SmartDashboard.putString("passthrough/command", current.getName());}
      SmartDashboard.putNumber("passthrough/Cargo In Robot", numberOfCargo);
      SmartDashboard.putNumber("passthrough/PTEncoder", encoderPTFront.getPosition());

      // motorPTFront.set(ptPidFront.getOutput(encoderPTFront.getPosition(),setpoint));
    }
    @Override
    public void simulationPeriodic() {
        // This method   will be called once per scheduler run during simulation
    }
}

