package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeMotorSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.limitSwitchDigitalPort);

	public IntakeMotorSubsystem() {
		m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
		m_intakeMotor.restoreFactoryDefaults();
	}

	public void setIntakeMotor(double speed) {
		if (!limitSwitch.get() && speed > 0) {
			m_intakeMotor.set(speed);
		}
	}

	public boolean currentlyAcquired() {
		return limitSwitch.get();
	}

	public Command enableIntake() {
		return this.runOnce(() -> m_intakeMotor.set(0.2));
	}

	public Command disableIntake() {
		return this.runOnce(() -> m_intakeMotor.set(0));
	}

	@Override
	public void periodic() {
		if (limitSwitch.get()) {
			m_intakeMotor.stopMotor();
		}

	}
}
