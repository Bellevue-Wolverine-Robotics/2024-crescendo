package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/*
 * Command that moves the flywheel arm to aim towards the intake
 */
public class FlywheelClimbModeCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelClimbModeCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	public void initialize() {
		m_flywheelSubsystem.climbMode();
	}

	public boolean isFinished() {
		return m_flywheelSubsystem.isArmClimbMode();
	}

}
