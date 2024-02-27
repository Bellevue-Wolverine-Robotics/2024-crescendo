package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends Command {
	private ClimberSubsystem m_climberSubsystem;

	public ClimberExtendCommand(ClimberSubsystem climberSubsystem) {
		m_climberSubsystem = climberSubsystem;
		addRequirements(m_climberSubsystem);
	}

	@Override
	public void initialize() {
		m_climberSubsystem.extend();
	}

	@Override
	public boolean isFinished() {
		return m_climberSubsystem.isExtended();
	}

	@Override
	public void end(boolean interrupted) {
	}
}
