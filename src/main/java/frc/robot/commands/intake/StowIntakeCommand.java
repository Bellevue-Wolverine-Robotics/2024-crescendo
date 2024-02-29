package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StowIntakeCommand extends Command {
	private IntakeSubsystem m_intakeSubsystem;

	public StowIntakeCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.stowIntakeArm();
	}

	@Override
	public boolean isFinished() {
		return m_intakeSubsystem.isIntakeArmStowed();
	}
}
