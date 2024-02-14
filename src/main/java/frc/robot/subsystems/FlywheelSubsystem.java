package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
	private CANSparkMax m_flywheelMotor;
	private SparkPIDController m_flywheelPidController;
	private RelativeEncoder m_flywheelEncoder;

	public FlywheelSubsystem() {
		// m_flywheelMotor = new CANSparkMax(FlywheelConstants.kFlywheelId,
		// MotorType.kBrushless);

		// m_flywheelMotor.restoreFactoryDefaults();

		// m_flywheelPidController = m_flywheelMotor.getPIDController();
		// m_flywheelEncoder = m_flywheelMotor.getEncoder();

		// m_flywheelPidController.setP(FlywheelConstants.kP);
		// m_flywheelPidController.setI(FlywheelConstants.kI);
		// m_flywheelPidController.setD(FlywheelConstants.kD);
		// m_flywheelPidController.setIZone(FlywheelConstants.kIZone);
		// m_flywheelPidController.setFF(FlywheelConstants.kFF);
		// m_flywheelPidController.setOutputRange(FlywheelConstants.kMinOutput,
		// FlywheelConstants.kMaxOutput);
	}

	public void setFlywheelVelocity(double setpoint) {
		// SmartDashboard.putNumber("Setpoint", setpoint);
		// SmartDashboard.putNumber("Flywheel Velocity",
		// m_flywheelEncoder.getVelocity());
		// SmartDashboard.putNumber("Error", setpoint -
		// m_flywheelEncoder.getVelocity());

		// m_flywheelPidController.setReference(setpoint, ControlType.kVelocity);
	}

	public void StartFlywheel() {
		m_flywheelMotor.set(1);
	}

	public void StopFlywheel() {
		m_flywheelMotor.set(0);
	}

	@Override
	public void periodic() {
	}
}
