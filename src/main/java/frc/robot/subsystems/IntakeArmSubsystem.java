package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeArm;
	private SparkPIDController m_intakePidController;
	private RelativeEncoder m_intakeArmRelativeEncoder;

	public IntakeArmSubsystem() {
		// m_intakeArm = new CANSparkMax(IntakeConstants.kIntakeArmId,
		// MotorType.kBrushless);

		// m_intakeArm.restoreFactoryDefaults();
		// m_intakeArm.setIdleMode(IdleMode.kBrake);
		// m_intakeArm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

		// m_intakeArmRelativeEncoder = m_intakeArm.getEncoder();
		// m_intakeArmRelativeEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);
		// m_intakeArmRelativeEncoder.setPosition(0);

		// m_intakePidController = m_intakeArm.getPIDController();
		// m_intakePidController.setP(IntakeConstants.kIntakeArmP);
		// m_intakePidController.setI(IntakeConstants.kIntakeArmI);
		// m_intakePidController.setD(IntakeConstants.kIntakeArmD);
		// m_intakePidController.setIZone(IntakeConstants.kIntakeArmIZone);
		// m_intakePidController.setFF(IntakeConstants.kIntakeArmFF);
		// m_intakePidController.setOutputRange(IntakeConstants.kIntakeArmMinOutput,
		// IntakeConstants.kIntakeArmMaxOutput);
	}

	public void setIntakeArm(double speed) {
		m_intakeArm.set(speed);
	}

	public void setIntakeArmVoltage(double voltage) {
		m_intakeArm.setVoltage(voltage);
	}

	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	@Override
	public void periodic() {
	}

	public Command goToAngle(double setpoint) {
		double ffTerm = IntakeConstants.kIntakeArmFFGravity * Math.cos(getAngle()); // should be in degrees also should
																					// be angel
		// taken from horizontal

		return this.runOnce(() -> m_intakePidController.setReference(setpoint, ControlType.kPosition, 0, ffTerm));
	}
}