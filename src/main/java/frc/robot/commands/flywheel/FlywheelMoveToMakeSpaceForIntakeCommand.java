package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Command that moves the flywheel arm to aim towards the stage
 */

public class FlywheelMoveToMakeSpaceForIntakeCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelMoveToMakeSpaceForIntakeCommand(FlywheelSubsystem flywheelSubsystem) {
		//System.out.println("FlywheelMoveToMakeSpaceForIntakeCommand");
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.aimArmToMakeSpaceForIntake();
	}

	@Override
	public boolean isFinished() {
		//if(m_flywheelSubsystem.isArmMakingSpaceForIntake())System.out.println("FlywheelMoveToMakeSpaceForIntakeCommand terminates");

		return m_flywheelSubsystem.isArmMakingSpaceForIntake();
	}
}
