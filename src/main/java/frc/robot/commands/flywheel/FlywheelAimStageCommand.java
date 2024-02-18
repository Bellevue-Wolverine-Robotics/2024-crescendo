package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Command that moves the flywheel arm to aim towards the stage
 */

public class FlywheelAimStageCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelAimStageCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		// m_flywheelSubsystem.aimAtIntake();
	}
}
