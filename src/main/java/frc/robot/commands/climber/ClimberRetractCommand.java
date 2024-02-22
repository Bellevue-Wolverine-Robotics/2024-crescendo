package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRetractCommand extends Command {
	private ClimberSubsystem m_climberSubsystem;

	public ClimberRetractCommand(ClimberSubsystem climberSubsystem) {
		m_climberSubsystem = climberSubsystem;
		addRequirements(m_climberSubsystem);
	}

	@Override
	public void initialize() {
		m_climberSubsystem.retract();
	}

	@Override
	public boolean isFinished() {
		return m_climberSubsystem.isRetracted();
	}

	@Override
	public void end(boolean interrupted) {
		m_climberSubsystem.holdPosition();
	}
}
