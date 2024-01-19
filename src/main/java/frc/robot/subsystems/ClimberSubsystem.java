package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.Throttles;

public class ClimberSubsystem extends SubsystemBase{
    private CANSparkMax climbMotor1 = new CANSparkMax(CANConstants.climbMotor1, MotorType.kBrushless);
    private CANSparkMax climbMotor2 = new CANSparkMax(CANConstants.climbMotor2, MotorType.kBrushless);

    private RelativeEncoder climberEncoder = climbMotor1.getEncoder();

    public ClimberSubsystem(){
        this.climbMotor1.restoreFactoryDefaults();
        this.climbMotor2.restoreFactoryDefaults();

        this.climbMotor1.setSmartCurrentLimit(30);
        this.climbMotor2.setSmartCurrentLimit(30);
    
        //Motor 1 and Motor 2 spin opposite direction.
        climbMotor2.follow(climbMotor1, true);

        climberEncoder.setPosition(0.0);
        climberEncoder.setPositionConversionFactor(PhysicalConstants.WHEEL_CIRCUMFERENCE_METERS / PhysicalConstants.DRIVE_GEAR_RATIO);
        
    }

    public double getPosition(){
        return this.climberEncoder.getPosition();
    }

    
    public void setSpeed(double speed)
    //@requires 0.0 <= speed && speed <= 1.0;
    {
        speed = speed > 1.0? 1.0: speed;
        speed = speed < 0.0? 0.0: speed;
        climbMotor1.setVoltage(speed*Throttles.climberVoltage);
    }





    
}
