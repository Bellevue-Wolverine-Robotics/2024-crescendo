package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.utils.PIDUtils;
import frc.utils.PIDUtils.ArmFFParams;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeArm;
	private SparkPIDController m_armPidController;
	private RelativeEncoder m_intakeArmRelativeEncoder;
	private ArmFeedforward m_armFeedforward;

	private CANSparkMax m_feederMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.kNoteSwitchDIOPort);

	public IntakeSubsystem() {
		m_feederMotor = new CANSparkMax(IntakeConstants.kFeederMotorId, MotorType.kBrushless);
		m_intakeArm = new CANSparkMax(IntakeConstants.kArmMotorId,
				MotorType.kBrushless);

		m_intakeArm.restoreFactoryDefaults();
		m_intakeArm.setIdleMode(IdleMode.kBrake);
		m_intakeArm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

		m_intakeArmRelativeEncoder = m_intakeArm.getEncoder();
		m_intakeArmRelativeEncoder.setPositionConversionFactor(IntakeConstants.kArmPositionConversionFactor);
		m_intakeArmRelativeEncoder.setPosition(0);

		m_armPidController = m_intakeArm.getPIDController();
		PIDUtils.setPIDConstants(m_armPidController, IntakeConstants.kIntakeArmPIDParams);

		m_armFeedforward = PIDUtils.createArmFeedforward(IntakeConstants.kIntakeArmFFParams);
	}

	// -- Intake Arm -- //
	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public void setIntakeArmPIDSetpoint(double setpoint) {
		double gravityFFTerm = m_armFeedforward.calculate(setpoint, 0, 0); // should be in degrees also should

		m_armPidController.setReference(setpoint, ControlType.kPosition, 0, gravityFFTerm);
	}

	public void deployIntakeArm() {
		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointDeploy);
	}

	public void stowIntakeArm() {
		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointStow);
	}

	// -- Intake Motor -- //
	public void setIntakeMotorSpeed(double speed) {
		if (!limitSwitch.get() && speed > 0) {
			m_feederMotor.set(speed);
		}
	}

	public void startIntakeMotor() {
		setIntakeMotorSpeed(IntakeConstants.kFeederDutyCycle);
	}

	public void stopIntakeMotor() {
		m_feederMotor.stopMotor();
	}

	public boolean hasNote() {
		return limitSwitch.get();
	}

	@Override
	public void periodic() {
		if (limitSwitch.get()) {
			m_feederMotor.stopMotor();
		}
	}
}