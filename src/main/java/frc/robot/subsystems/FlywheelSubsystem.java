package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PIDConstants.FlywheelPID;

public class FlywheelSubsystem extends SubsystemBase {
	private CANSparkMax m_flywheelMotor;
	private SparkPIDController m_flywheelPidController;
	private RelativeEncoder m_flywheelEncoder;

	public FlywheelSubsystem() {
		m_flywheelMotor = new CANSparkMax(CANConstants.kFlywheelId, MotorType.kBrushless);

		m_flywheelMotor.restoreFactoryDefaults();

		m_flywheelPidController = m_flywheelMotor.getPIDController();
		m_flywheelEncoder = m_flywheelMotor.getEncoder();

		m_flywheelPidController.setP(FlywheelPID.kP);
		m_flywheelPidController.setI(FlywheelPID.kI);
		m_flywheelPidController.setD(FlywheelPID.kD);
		m_flywheelPidController.setIZone(FlywheelPID.kIZone);
		m_flywheelPidController.setFF(FlywheelPID.kFF);
		m_flywheelPidController.setOutputRange(FlywheelPID.kMinOutput, FlywheelPID.kMaxOutput);
	}

	public void setFlywheelVelocity(double setpoint) {
		System.out.println("Setpoint: " + setpoint);

		SmartDashboard.putNumber("Setpoint", setpoint);
		SmartDashboard.putNumber("Flywheel Velocity", m_flywheelEncoder.getVelocity());
		SmartDashboard.putNumber("Error", setpoint - m_flywheelEncoder.getVelocity());

		m_flywheelPidController.setReference(setpoint, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		System.out.println("Flywheel Velocity: " + m_flywheelEncoder.getVelocity());
	}
}
