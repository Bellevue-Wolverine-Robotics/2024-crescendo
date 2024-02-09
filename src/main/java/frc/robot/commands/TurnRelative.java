package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRelative extends Command {
	private DriveSubsystem m_driveSubsystem;
	private int setPoint;
	private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn,
            DriveConstants.kDTurn);

	public TurnRelative(int radians, DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
		setPoint = radians;
	}

	@Override
	public void execute() {
		
	}
}
