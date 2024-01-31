package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;

public class IntakeArm extends Command {
	private IntakeArmSubsystem m_intakeArmSubsystem;
	private ArmFeedforward m_feedForward;
	private PIDController m_controller;

	private double m_setpoint;

	public IntakeArm(IntakeArmSubsystem intakeArmSubsystem, double setpoint) {
		m_intakeArmSubsystem = intakeArmSubsystem;

		m_feedForward = new ArmFeedforward(0.0, IntakeConstants.kIntakeArmFFGravity, 0.0, 0.0);
		m_controller = new PIDController(IntakeConstants.kIntakeArmP,
				IntakeConstants.kIntakeArmI,
				IntakeConstants.kIntakeArmD);

		m_setpoint = setpoint;

		addRequirements(m_intakeArmSubsystem);
	}

	@Override
	public void execute() {
		m_intakeArmSubsystem.setIntakeArmVoltage(m_controller.calculate(m_intakeArmSubsystem.getAngle(), m_setpoint)
				+ m_feedForward.calculate(m_intakeArmSubsystem.getAngle(), 0));
	}

	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		m_intakeArmSubsystem.setIntakeArmVoltage(0.0);
	}

}
