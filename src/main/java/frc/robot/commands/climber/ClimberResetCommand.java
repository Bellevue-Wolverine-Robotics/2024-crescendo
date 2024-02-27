package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberResetCommand extends Command {

	private ClimberSubsystem m_climberSubsystem;

	public ClimberResetCommand(ClimberSubsystem climberSubsystem) {
		m_climberSubsystem = climberSubsystem;

		addRequirements(climberSubsystem);
	}

	@Override
	public void execute() {
		m_climberSubsystem.setDutyCycle(ClimberConstants.kClimberResetSpeed);
	}

	@Override
	public boolean isFinished() {
		return m_climberSubsystem.limitSwitchVelocityCheck();
	}

	@Override
	public void end(boolean interrupted) {
		m_climberSubsystem.stopMotors();
	}
}
