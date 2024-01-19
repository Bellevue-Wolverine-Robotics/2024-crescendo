package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climbing;
import frc.robot.commands.Climb;

public class ClimberSubsystem extends SubsystemBase{
    private CANSparkMax climbMotor1 = new CANSparkMax(Climbing.climbMotor1, MotorType.kBrushless);
    private CANSparkMax climbMotor2 = new CANSparkMax(Climbing.climbMotor2, MotorType.kBrushless);
    private DigitalInput limitSwitch = new DigitalInput(Climbing.limitSwitchDigitalPort);

    private boolean limitSwitchEnabled = true;
    private RelativeEncoder climberEncoder = climbMotor1.getEncoder();

    public ClimberSubsystem(){
        this.climbMotor1.restoreFactoryDefaults();
        this.climbMotor2.restoreFactoryDefaults();

        this.climbMotor1.setIdleMode(IdleMode.kBrake);
        this.climbMotor2.setIdleMode(IdleMode.kBrake);

        this.climbMotor1.setSmartCurrentLimit(30);
        this.climbMotor2.setSmartCurrentLimit(30);
    
        //Motor 1 and Motor 2 spin opposite direction.
        climbMotor2.follow(climbMotor1, true);

        climberEncoder.setPosition(0.0);
        climberEncoder.setPositionConversionFactor(Climbing.WHEEL_CIRCUMFERENCE_METERS / Climbing.DRIVE_GEAR_RATIO);
    }

    public double getPosition(){
        return this.climberEncoder.getPosition();
    }

    
    public void setSpeed(double speed)
    //@requires 0.0 <= speed && speed <= 1.0;
    {
        if(limitSwitchEnabled && limitSwitch.get()){
            climbMotor1.setVoltage(0.0);
        }
        else{
            climbMotor1.setVoltage(MathUtil.clamp(speed, 0.0, 1.0)*Climbing.climberVoltage);
        }
    }


    public void stop(){
        climbMotor1.set(0.0);
    }




    
}
