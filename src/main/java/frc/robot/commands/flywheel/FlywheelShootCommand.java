package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import java.lang.Math;
import frc.robot.constants.FlywheelConstants;

public class FlywheelShootCommand extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelShootCommand(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.startShooter();
	}

	@Override
	public boolean isFinished() {
		//return false

		
		double targetVelocity = FlywheelConstants.kShootSpeakerDutyCycleSetpoint;
		double currentVelocity = m_flywheelSubsystem.getShooterVelocity ();
		double error = FlywheelConstants.kShooterVelocityTolerance;

		return Math.abs(targetVelocity - currentVelocity) < error;
	}

	@Override
	public void end(boolean interrupted) {
		m_flywheelSubsystem.stopShooter();
	}
}
