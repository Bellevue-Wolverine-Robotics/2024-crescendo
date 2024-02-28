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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.IntakeConstants;
import frc.utils.PIDUtils;
import frc.utils.PIDUtils.ArmFFParams;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax m_intakeArm;
	private RelativeEncoder m_intakeArmRelativeEncoder;
	private ArmFeedforward m_armFeedforward;
	private SparkPIDController m_intakeArmPID;

	private WPI_TalonSRX m_feederMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.kNoteSwitchDIOPort);

	public IntakeSubsystem() {
		m_feederMotor = new WPI_TalonSRX(IntakeConstants.kArmMotorId);
		m_intakeArm = new CANSparkMax(IntakeConstants.kFeederMotorId, MotorType.kBrushless);
		m_intakeArmPID = m_intakeArm.getPIDController();


		PIDUtils.setPIDConstants(m_intakeArm.getPIDController(), IntakeConstants.kIntakeArmPIDParams);

		m_armFeedforward = PIDUtils.createArmFeedforward(IntakeConstants.kIntakeArmFFParams);

		SmartDashboard.putNumber("m_intakeArmPID kP", m_intakeArmPID.getP());
		SmartDashboard.putNumber("m_intakeArmPID kI", m_intakeArmPID.getI());
		SmartDashboard.putNumber("m_intakeArmPID kD", m_intakeArmPID.getD());
		SmartDashboard.putNumber("m_intakeArmPID kD", m_intakeArmPID.getFF());
	}

	// -- Intake Arm -- //
	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public void setIntakeArmPIDSetpoint(double setpoint) {
		m_intakeArmPID.setReference(setpoint, ControlType.kPosition);
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

		var intakeArmParams = new PIDUtils.SparkPIDParams(m_intakeArm);
		intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kP", m_intakeArmPID.getP()));
		intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kI", m_intakeArmPID.getI()));
		intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kD", m_intakeArmPID.getD()));
		intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kD", m_intakeArmPID.getFF()));

		PIDUtils.setPIDConstants(m_intakeArmPID, intakeArmParams);


		if (limitSwitch.get()) {
			m_feederMotor.stopMotor();
		}
	}
}