package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeMotor;
	private CANSparkMax m_intakeArm;

	public IntakeSubsystem() {
		m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
		m_intakeArm = new CANSparkMax(IntakeConstants.kIntakeArmId, MotorType.kBrushless);

		m_intakeMotor.restoreFactoryDefaults();
	}

	public void enableIntake() {
		System.out.println("intake enabled");
		m_intakeMotor.set(1);
	}

	public void disableIntake() {
		m_intakeMotor.set(0);
	}

	@Override
	public void periodic() {
	}
}
