package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {
	private IntakeSubsystem m_intakeSubsystem;

	public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.startIntakeMotor();
		m_intakeSubsystem.deployIntakeArm();
	}

	@Override
	public boolean isFinished() {
		return m_intakeSubsystem.hasNote();
	}

	@Override
	public void end(boolean interrupted) {
		m_intakeSubsystem.stopIntakeMotor();
	}
}
