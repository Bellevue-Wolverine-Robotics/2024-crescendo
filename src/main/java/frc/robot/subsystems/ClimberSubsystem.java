package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftClimbMotor;
    private CANSparkMax m_rightClimbMotor;

    private SparkPIDController m_leftClimbPidController;
    private SparkPIDController m_rightClimbPidController;

    private RelativeEncoder m_leftClimbRelativeEncoder;
    private RelativeEncoder m_rightClimbRelativeEncoder;

    private DigitalInput limitSwitch = new DigitalInput(ClimbingConstants.limitSwitchDigitalPort);

    private boolean limitSwitchEnabled = true;

    public ClimberSubsystem() {
        // m_leftClimbMotor = new CANSparkMax(ClimbingConstants.kLeftClimbMotorId,
        // MotorType.kBrushless);
        // m_rightClimbMotor = new CANSparkMax(ClimbingConstants.kRightClimbMotorId,
        // MotorType.kBrushless);

        // // m_rightClimbMotor.follow(m_leftClimbMotor, true); // THIS DOESN'T WORK RN
        // IDK WHY

        // m_leftClimbRelativeEncoder = m_leftClimbMotor.getEncoder();
        // m_rightClimbRelativeEncoder = m_rightClimbMotor.getEncoder();

        // m_leftClimbPidController = m_leftClimbMotor.getPIDController();
        // m_rightClimbPidController = m_rightClimbMotor.getPIDController();

        // buildMotor(m_leftClimbMotor, false);
        // buildMotor(m_rightClimbMotor, true);

        // buildRelativeEncoder(m_leftClimbRelativeEncoder, false);
        // buildRelativeEncoder(m_rightClimbRelativeEncoder, true);

        // buildPidController(m_leftClimbPidController);
        // buildPidController(m_rightClimbPidController);
    }

    private void buildMotor(CANSparkMax motor, boolean inverted) {
        motor.restoreFactoryDefaults();

        motor.setInverted(inverted);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ClimbingConstants.kSmartCurrentLimit);
    }

    private void buildRelativeEncoder(RelativeEncoder encoder, boolean inverted) {
        encoder.setPosition(0.0);
        encoder.setPositionConversionFactor(
                inverted ? ClimbingConstants.kPositionConversionFactor : -ClimbingConstants.kPositionConversionFactor);
    }

    private void buildPidController(SparkPIDController pidController) {
        pidController.setP(ClimbingConstants.kP);
        pidController.setI(ClimbingConstants.kI);
        pidController.setD(ClimbingConstants.kD);
        pidController.setIZone(ClimbingConstants.kIZone);
        pidController.setFF(ClimbingConstants.kFF);
        pidController.setOutputRange(ClimbingConstants.kMinOutput, ClimbingConstants.kMaxOutput);
    }

    public double getPosition() {
        return m_leftClimbRelativeEncoder.getPosition(); // ideally returns both
    }

    public void setSpeed(double speed)
    // @requires 0.0 <= speed && speed <= 1.0;
    {
        if (limitSwitchEnabled && limitSwitch.get()) {
            m_leftClimbMotor.setVoltage(0.0);
        } else {
            m_leftClimbMotor.setVoltage(MathUtil.clamp(speed, 0.0, 1.0) * ClimbingConstants.climberVoltage);
        }
    }

    public void setVoltageRaw(double rawVoltage) {
        // m_leftClimbMotor.setVoltage(rawVoltage);
        // m_rightClimbMotor.setVoltage(rawVoltage);
    }

    public Command climbUpCommand() {
        // return this.runOnce(() -> setSpeed(ClimbingConstants.kOperatorClimbSpeed));
        return this.startEnd(() -> setVoltageRaw(1), () -> stopMotors());
    }

    public Command climbDownCommand() {
        // return this.runOnce(() -> setSpeed(-ClimbingConstants.kOperatorClimbSpeed));
        return this.startEnd(() -> setVoltageRaw(-1), () -> stopMotors());
    }

    public Command climbToPositionSetpointCommand(double setpoint) {
        return this.runOnce(() -> {
            m_leftClimbPidController.setReference(setpoint, ControlType.kPosition);
            m_rightClimbPidController.setReference(setpoint, ControlType.kPosition);
        });
    }

    public void stopMotors() {
        m_leftClimbMotor.set(0.0);
        m_rightClimbMotor.set(0.0);
    }
}
