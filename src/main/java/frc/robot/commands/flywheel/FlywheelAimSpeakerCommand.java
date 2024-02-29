package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Command that moves the flywheel arm to aim towards the stage
 */

public class FlywheelAimSpeakerCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelAimSpeakerCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.aimArmToSpeaker();
	}

	@Override
	public boolean isFinished() {
		return m_flywheelSubsystem.isArmAimingTowardsSpeaker();
	}
}
