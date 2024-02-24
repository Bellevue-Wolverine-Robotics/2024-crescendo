package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.utils.PIDUtils;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_leftClimbMotor;
    private CANSparkMax m_rightClimbMotor;

    private SparkPIDController m_leftClimbPidController;
    private SparkPIDController m_rightClimbPidController;

    private RelativeEncoder m_leftClimbRelativeEncoder;
    private RelativeEncoder m_rightClimbRelativeEncoder;

    private DigitalInput m_topLimitSwitch = new DigitalInput(ClimberConstants.kTopLimitSwitchDIOPort);
    private DigitalInput m_bottomLimitSwitch = new DigitalInput(ClimberConstants.kBottomLimitSwitchDIO);

    private boolean m_limitSwitchEnabled = true;

    public ClimberSubsystem() {
        m_leftClimbMotor = new CANSparkMax(ClimberConstants.kLeftClimbMotorId,
                MotorType.kBrushless);
        m_rightClimbMotor = new CANSparkMax(ClimberConstants.kRightClimbMotorId,
                MotorType.kBrushless);

        m_leftClimbRelativeEncoder = m_leftClimbMotor.getEncoder();
        m_rightClimbRelativeEncoder = m_rightClimbMotor.getEncoder();

        m_leftClimbPidController = m_leftClimbMotor.getPIDController();
        m_rightClimbPidController = m_rightClimbMotor.getPIDController();

        buildMotor(m_leftClimbMotor, false);
        buildMotor(m_rightClimbMotor, true);

        m_leftClimbRelativeEncoder.setPosition(0);
        m_rightClimbRelativeEncoder.setPosition(0);

        m_leftClimbRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversion);
        m_rightClimbRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversion);

        PIDUtils.setPIDConstants(m_leftClimbPidController, ClimberConstants.kClimbPidParams);
        PIDUtils.setPIDConstants(m_rightClimbPidController, ClimberConstants.kClimbPidParams);
    }

    private void buildMotor(CANSparkMax motor, boolean inverted) {
        motor.restoreFactoryDefaults();

        motor.setInverted(inverted);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
    }

    public Pair<Double, Double> getPosition() {
        return new Pair<>(m_leftClimbRelativeEncoder.getPosition(), m_rightClimbRelativeEncoder.getPosition());
    }

    // Positive voltage/duty cycle corresponds to raising the elevator
    public void setDutyCycle(double dutyCycle)
    // @requires 0.0 <= dutyCycle && dutyCycle <= 1.0;
    {
        if (limitSwitchSetCheck(dutyCycle))
            return;

        m_leftClimbMotor.set(dutyCycle);
        m_rightClimbMotor.set(dutyCycle);
    }

    public void setVoltage(double voltage) {
        if (limitSwitchSetCheck(voltage))
            return;

        m_leftClimbMotor.setVoltage(voltage);
        m_rightClimbMotor.setVoltage(voltage);
    }

    private boolean limitSwitchSetCheck(double value) {
        return m_bottomLimitSwitch.get() && value < 0 || m_topLimitSwitch.get() && value > 0;
    }

    public boolean limitSwitchVelocityCheck() {
        return m_bottomLimitSwitch.get() && m_leftClimbRelativeEncoder.getVelocity() < 0
                || m_topLimitSwitch.get() && m_leftClimbRelativeEncoder.getVelocity() > 0;
    }

    public void setClimbPIDPositionSetpoint(double positionSetpoint) {
        m_leftClimbPidController.setReference(positionSetpoint, ControlType.kPosition);
        m_rightClimbPidController.setReference(positionSetpoint, ControlType.kPosition);
    }

    public void retract() {
        setClimbPIDPositionSetpoint(ClimberConstants.kClimbRetractedSetpoint);
    }

    public void extend() {
        setClimbPIDPositionSetpoint(ClimberConstants.kClimbExtendedSetpoint);
    }

    public boolean atSetpoint(double setpoint) {
        return PIDUtils.atSetpoint(m_leftClimbRelativeEncoder.getPosition(), setpoint,
                ClimberConstants.kClimbTolerance)
                && PIDUtils.atSetpoint(m_rightClimbRelativeEncoder.getPosition(), setpoint,
                        ClimberConstants.kClimbTolerance);
    }

    public boolean isExtended() {
        return atSetpoint(ClimberConstants.kClimbExtendedSetpoint);
    }

    public boolean isRetracted() {
        return atSetpoint(ClimberConstants.kClimbRetractedSetpoint);
    }

    public void stopMotors() {
        m_leftClimbMotor.stopMotor();
        m_rightClimbMotor.stopMotor();
    }

    public void holdPosition() {
        setClimbPIDPositionSetpoint(m_rightClimbRelativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition().getFirst());

        SmartDashboard.putNumber("Left Encoder Velocity",
                m_leftClimbRelativeEncoder.getVelocity());

        SmartDashboard.putNumber("Right Encoder Velocity",
                m_rightClimbRelativeEncoder.getVelocity());

        if (m_limitSwitchEnabled && limitSwitchVelocityCheck()) {
            this.stopMotors();
        }

        SmartDashboard.putNumber("Climber Conversion Factor", ClimberConstants.kClimberPositionConversion);
        SmartDashboard.putNumber("kSprocketDiameterInches", ClimberConstants.kSprocketDiameterInches);
        SmartDashboard.putNumber("kSprocketDiameterMeters", ClimberConstants.kSprocketDiameterMeters);
        SmartDashboard.putNumber("kSprocketCircumferenceMeters", ClimberConstants.kSprocketCircumferenceMeters);

    }
}