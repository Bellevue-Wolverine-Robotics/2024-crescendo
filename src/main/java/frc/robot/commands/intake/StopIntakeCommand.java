package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends Command {
	private IntakeSubsystem m_intakeSubsystem;

	public StopIntakeCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.stopIntakeMotor();
		m_intakeSubsystem.stowIntakeArm();
	}
}
