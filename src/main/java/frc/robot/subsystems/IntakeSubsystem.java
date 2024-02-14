package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeArm;
	private SparkPIDController m_intakePidController;
	private RelativeEncoder m_intakeArmRelativeEncoder;
	private ArmFeedforward m_armFeedforward;

	private CANSparkMax m_intakeMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.limitSwitchDigitalPort);

	public IntakeSubsystem() {
		m_intakeArm = new CANSparkMax(IntakeConstants.kIntakeArmId,
				MotorType.kBrushless);

		m_intakeArm.restoreFactoryDefaults();
		m_intakeArm.setIdleMode(IdleMode.kBrake);
		m_intakeArm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

		m_intakeArmRelativeEncoder = m_intakeArm.getEncoder();
		m_intakeArmRelativeEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);
		m_intakeArmRelativeEncoder.setPosition(0);

		m_intakePidController = m_intakeArm.getPIDController();
		m_intakePidController.setP(IntakeConstants.kIntakeArmP);
		m_intakePidController.setI(IntakeConstants.kIntakeArmI);
		m_intakePidController.setD(IntakeConstants.kIntakeArmD);
		m_intakePidController.setIZone(IntakeConstants.kIntakeArmIZone);
		m_intakePidController.setFF(IntakeConstants.kIntakeArmFF);
		m_intakePidController.setOutputRange(IntakeConstants.kIntakeArmMinOutput,
				IntakeConstants.kIntakeArmMaxOutput);

		m_armFeedforward = new ArmFeedforward(IntakeConstants.kIntakeArmFFStatic, IntakeConstants.kIntakeArmFFGravity,
				IntakeConstants.kIntakeArmFFVelocity, IntakeConstants.kIntakeArmFFAcceleration);
	}

	// -- Intake Arm -- //
	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public void setIntakeArmPIDSetpoint(double setpoint) {
		double gravityFFTerm = m_armFeedforward.calculate(setpoint, 0, 0); // should be in degrees also should

		m_intakePidController.setReference(setpoint, ControlType.kPosition, 0, gravityFFTerm);
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
			m_intakeMotor.set(speed);
		}
	}

	public void startIntakeMotor() {
		setIntakeMotorSpeed(IntakeConstants.intakeMotorSpeed);
	}

	public void stopIntakeMotor() {
		m_intakeMotor.stopMotor();
	}

	public boolean hasNote() {
		return limitSwitch.get();
	}

	@Override
	public void periodic() {
		if (limitSwitch.get()) {
			m_intakeMotor.stopMotor();
		}
	}
}