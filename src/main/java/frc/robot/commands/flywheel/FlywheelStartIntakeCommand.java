package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStartIntakeCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelStartIntakeCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		// m_flywheelSubsystem.startIntakeMotor();
	}
}
