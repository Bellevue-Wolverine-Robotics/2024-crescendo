package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
	private TalonSRX m_flywheelLeft;
	private TalonSRX m_flywheelRight;
	private SparkPIDController m_flywheelPidController;
	private Spark
	private RelativeEncoder m_flywheelEncoder;

	public flyWheelSpeed (int speed) {
		//set speed of both flywheels
	}

	public 

	public FlywheelSubsystem() {

		m_flywheelLeft = new TalonSRX (Constants.ArmConstants.FlyWheel.leftNumber);
		m_flywheelRight = new TalonSRX (Constants.ArmConstants.FlyWheel.rightNumber);

		

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
