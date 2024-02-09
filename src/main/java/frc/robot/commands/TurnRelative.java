package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRelative extends Command {
	private DriveSubsystem m_driveSubsystem;
	private double setPoint;
	private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn,
			DriveConstants.kDTurn);

	public TurnRelative(double radians, DriveSubsystem driveSubsystem)
	// @requires -Math.PI <= radians && radians <= Math.PI
	// @requires driveSubsystem != null
	{
		m_driveSubsystem = driveSubsystem;
		setPoint = Math.toRadians(m_driveSubsystem.getYaw()) + radians;

		m_pid.setTolerance(0.1);

		addRequirements(m_driveSubsystem);
	}

	@Override
	public void execute() {
		double motorSpeed = m_pid.calculate(m_driveSubsystem.getYaw(), setPoint);
		m_driveSubsystem.tankDrive(-motorSpeed, motorSpeed);
	}

	@Override
	public boolean isFinished() {
		return false;
		// return m_pid.atSetpoint();
	}

	@Override
	public void end(boolean interrupted	) {
		m_driveSubsystem.stopDriveTrain();
	}
}
