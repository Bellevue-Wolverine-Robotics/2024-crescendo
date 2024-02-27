package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import java.lang.Math;

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

		/*
		double targetVelocity = constants.FlywheelConstants.kShootSpeakerDutyCycleSetpoint;
		double currentVelocity = m_flywheelSubsystem.getShooterVelocity ();
		double error = constants.FlywheelConstants.kShooterVelocityTolerance;
		*/

		//all statements in one line to minimize memory usage
		return Math.abs (m_flywheelSubsystem.getShooterVelocity () - constants.FlywheelConstants.kShootSpeakerDutyCycleSetpoint) < constants.FlywheelConstants.kShooterVelocityTolerance;
	}

	@Override
	public void end(boolean interrupted) {
		m_flywheelSubsystem.stopShooter();
	}
}
