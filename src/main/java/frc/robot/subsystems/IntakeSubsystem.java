package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.IntakeConstants;
import frc.utils.PIDUtils;
import frc.utils.PIDUtils.ArmFFParams;

public class IntakeSubsystem extends SubsystemBase {
	private WPI_TalonSRX m_intakeArm;
	private RelativeEncoder m_intakeArmRelativeEncoder;
	private ArmFeedforward m_armFeedforward;

	private CANSparkMax m_feederMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.kNoteSwitchDIOPort);

	public IntakeSubsystem() {
		m_feederMotor = new CANSparkMax(IntakeConstants.kFeederMotorId, MotorType.kBrushless);
		m_intakeArm = new WPI_TalonSRX(IntakeConstants.kArmMotorId);

		PIDUtils.setPIDConstants(m_intakeArm, IntakeConstants.kIntakeArmPIDParams);

		m_armFeedforward = PIDUtils.createArmFeedforward(IntakeConstants.kIntakeArmFFParams);
	}

	// -- Intake Arm -- //
	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public void setIntakeArmPIDSetpoint(double setpoint) {
		double gravityFFTerm = m_armFeedforward.calculate(setpoint, 0, 0); // should be in degrees also should

		m_intakeArm.set(TalonSRXControlMode.Position, setpoint);
	}

	public void deployIntakeArm() {
		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointDeploy);
	}

	public void stowIntakeArm() {
		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointStow);
	}

	// -- Intake Motor -- //
	public void setIntakeMotorSpeed(double speed) {
		if (!limitSwitch.get()) {
			m_feederMotor.set(speed);
		}
	}

	// -- SHOOTER MODE -- //
	public void shoot() {
		setIntakeMotorSpeed(-1.0);
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