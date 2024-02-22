package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FlywheelStopIntakeCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelStopIntakeCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		// m_flywheelSubsystem.stopIntakeMotor();
	}
}
