package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlywheelConstants;
import frc.utils.PIDUtils;

public class FlywheelSubsystem extends SubsystemBase {
	private TalonSRX m_shooterMotorLeader;
	private TalonSRX m_shooterMotorFollower;

	private SparkPIDController m_shooterPidController;
	private RelativeEncoder m_shooterEncoder;

	private CANSparkMax m_armShoulderMotor;
	private CANSparkMax m_armElbowMotor;

	private SparkPIDController m_armShoulderPidController;
	private SparkPIDController m_armElbowPidController;

	private TalonSRX m_feederMotor; // this is a motor that collects the note from the intake

	public FlywheelSubsystem() {
		// Shooter Init
		m_shooterMotorLeader = new TalonSRX(FlywheelConstants.kShooterLeaderId);
		m_shooterMotorFollower = new TalonSRX(FlywheelConstants.kShooterFollowerId);

		m_shooterMotorFollower.follow(m_shooterMotorLeader);

		// TODO: Set up PID for Talon

		// Arm Init
		m_armShoulderMotor = new CANSparkMax(FlywheelConstants.kArmShoulderId, MotorType.kBrushless);
		m_armElbowMotor = new CANSparkMax(FlywheelConstants.kArmElbowId, MotorType.kBrushless);

		m_armShoulderMotor.restoreFactoryDefaults();
		m_armElbowMotor.restoreFactoryDefaults();

		m_armShoulderPidController = m_armShoulderMotor.getPIDController();
		m_armElbowPidController = m_armElbowMotor.getPIDController();

		PIDUtils.setPIDConstants(m_armShoulderPidController, FlywheelConstants.kArmShoulderPid);
		PIDUtils.setPIDConstants(m_armElbowPidController, FlywheelConstants.kArmElbowPid);
	}

	public void setShooterVelocity(double setpoint) {
		// TODO: Implement this method
	}

	public void setShooterDutyCycle(double dutyCycle) {
		m_shooterMotorLeader.set(TalonSRXControlMode.PercentOutput, dutyCycle);
	}

	public void startShooter() {
		setShooterDutyCycle(FlywheelConstants.kShootStageDutyCycleSetpoint);
	}

	public void stopShooter() {
		setShooterDutyCycle(0);
	}

	public void setArmSetpoint(double shoulderSetpoint, double elbowSetpoint) {
		m_armShoulderPidController.setReference(shoulderSetpoint, ControlType.kPosition);
		m_armElbowPidController.setReference(elbowSetpoint, ControlType.kPosition);
	}

	public void aimArmToStage() {
		setArmSetpoint(FlywheelConstants.kStageShoulderSetpoint, FlywheelConstants.kStageElbowSetpoint);
	}

	public void aimArmToAmp() {
		setArmSetpoint(FlywheelConstants.kAmpShoulderSetpoint, FlywheelConstants.kAmpElbowSetpoint);
	}

	public void feedIntoShooter() {
		m_feederMotor.set(TalonSRXControlMode.PercentOutput, 1);
	}

	@Override
	public void periodic() {
	}
}
