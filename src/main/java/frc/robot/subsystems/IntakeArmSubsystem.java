package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeArm;
	private RelativeEncoder m_intakeArmRelativeEncoder;

	public IntakeArmSubsystem() {
		m_intakeArm = new CANSparkMax(IntakeConstants.kIntakeArmId, MotorType.kBrushless);

		m_intakeArm.restoreFactoryDefaults();
		m_intakeArm.setIdleMode(IdleMode.kBrake);
		m_intakeArm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

		m_intakeArmRelativeEncoder = m_intakeArm.getEncoder();
		m_intakeArmRelativeEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);
		m_intakeArmRelativeEncoder.setPosition(0);
	}

	public void setIntakeArm(double speed) {
		m_intakeArm.set(speed);
	}

	public void setIntakeArmVoltage(double voltage) {
		m_intakeArm.setVoltage(voltage);
	}

	public Command enableIntakeArm() {
		return this.runOnce(() -> m_intakeArm.set(0.2));
	}

	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public Command disableIntakeArm() {
		return this.runOnce(() -> m_intakeArm.set(0));
	}

	@Override
	public void periodic() {
	}
}