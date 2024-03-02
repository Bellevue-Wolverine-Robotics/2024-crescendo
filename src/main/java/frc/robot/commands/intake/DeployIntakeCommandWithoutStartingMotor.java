package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommandWithoutStartingMotor extends Command {
	private IntakeSubsystem m_intakeSubsystem;

	public DeployIntakeCommandWithoutStartingMotor(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.deployIntakeArm();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
