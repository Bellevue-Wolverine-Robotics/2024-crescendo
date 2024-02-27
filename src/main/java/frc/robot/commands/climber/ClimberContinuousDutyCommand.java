package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberContinuousDutyCommand extends Command {
	private CommandJoystick m_joystick;
	private ClimberSubsystem m_climberSubsystem;

	public ClimberContinuousDutyCommand(ClimberSubsystem climberSubsystem, CommandJoystick joystick) {
		m_climberSubsystem = climberSubsystem;
		m_joystick = joystick;
		addRequirements(m_climberSubsystem);
	}

	@Override
	public void execute() {
		m_climberSubsystem.setDutyCycle(-m_joystick.getY() * ClimberConstants.kControllerClimberSensitivity);
	}
}
