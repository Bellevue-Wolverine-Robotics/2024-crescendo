package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftClimbMotor;
    private CANSparkMax m_rightClimbMotor;

    private SparkPIDController m_leftClimbPidController;
    private SparkPIDController m_rightClimbPidController;

    private RelativeEncoder m_leftClimbRelativeEncoder;
    private RelativeEncoder m_rightClimbRelativeEncoder;

    private DigitalInput m_limitSwitch = new DigitalInput(ClimbingConstants.kLimitSwitchDIOPort);

    private boolean m_limitSwitchEnabled = true;

    public ClimberSubsystem() {
        m_leftClimbMotor = new CANSparkMax(ClimbingConstants.kLeftClimbMotorId,
                MotorType.kBrushless);
        m_rightClimbMotor = new CANSparkMax(ClimbingConstants.kRightClimbMotorId,
                MotorType.kBrushless);

        m_leftClimbRelativeEncoder = m_leftClimbMotor.getEncoder();
        m_rightClimbRelativeEncoder = m_rightClimbMotor.getEncoder();

        m_leftClimbPidController = m_leftClimbMotor.getPIDController();
        m_rightClimbPidController = m_rightClimbMotor.getPIDController();

        buildMotor(m_leftClimbMotor, false);
        buildMotor(m_rightClimbMotor, true);

        buildRelativeEncoder(m_leftClimbRelativeEncoder, false);
        buildRelativeEncoder(m_rightClimbRelativeEncoder, true);

        buildPidController(m_leftClimbPidController);
        buildPidController(m_rightClimbPidController);
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

    public Pair<Double, Double> getPosition() {
        return new Pair<>(m_leftClimbRelativeEncoder.getPosition(), m_rightClimbRelativeEncoder.getPosition());
    }

    public void setSpeed(double speed)
    // @requires 0.0 <= speed && speed <= 1.0;
    {
        m_leftClimbMotor.set(speed);
        m_rightClimbMotor.set(speed);
    }

    public void setVoltageRaw(double rawVoltage) {
        m_leftClimbMotor.setVoltage(rawVoltage);
        m_rightClimbMotor.setVoltage(rawVoltage);
    }

    public void setClimbPIDSetpoint(double setpoint) {
        m_leftClimbPidController.setReference(setpoint, ControlType.kPosition);
        m_rightClimbPidController.setReference(setpoint, ControlType.kPosition);
    }

    public void retract() {
        setClimbPIDSetpoint(ClimbingConstants.kClimbRetractedSetpoint);
    }

    public void extend() {
        setClimbPIDSetpoint(ClimbingConstants.kClimbExtendedSetpoint);
    }

    public boolean atSetpoint(double setpoint) {
        return m_leftClimbRelativeEncoder.getPosition() - setpoint < ClimbingConstants.kClimbTolerance &&
                m_rightClimbRelativeEncoder.getPosition() - setpoint < ClimbingConstants.kClimbTolerance;
    }

    public boolean isExtended() {
        return atSetpoint(ClimbingConstants.kClimbExtendedSetpoint);
    }

    public boolean isRetracted() {
        return atSetpoint(ClimbingConstants.kClimbRetractedSetpoint);
    }

    public void stopMotors() {
        m_leftClimbMotor.stopMotor();
        m_rightClimbMotor.stopMotor();
    }

    public void holdPosition() {
        setClimbPIDSetpoint(m_leftClimbRelativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
        if (m_limitSwitchEnabled && m_limitSwitch.get()) {
            this.stopMotors();
        }
    }
}