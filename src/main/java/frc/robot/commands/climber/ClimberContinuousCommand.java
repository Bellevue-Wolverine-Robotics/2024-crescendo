package frc.robot.commands.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberContinuousCommand extends Command {
	private CommandJoystick m_joystick;
	private ClimberSubsystem m_climberSubsystem;

	ClimberContinuousCommand(ClimberSubsystem climberSubsystem, CommandJoystick joystick) {
		m_climberSubsystem = climberSubsystem;
		m_joystick = joystick;
		addRequirements(m_climberSubsystem);
	}

	@Override
	public void execute() {
		m_climberSubsystem
				.setClimbPIDPositionSetpoint(
						MathUtil.clamp(m_joystick.getY() * ClimberConstants.kClimbRetractedSetpoint,
								ClimberConstants.kClimbRetractedSetpoint, ClimberConstants.kClimbExtendedSetpoint));
	}
}
