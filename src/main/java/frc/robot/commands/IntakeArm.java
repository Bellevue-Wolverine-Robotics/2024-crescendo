package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Debug;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArm extends Command {

	IntakeArmSubsystem m_intakeArmSubsystem;

	public IntakeArm(IntakeArmSubsystem m_intakeArmSubsystem) {
		this.m_intakeArmSubsystem = m_intakeArmSubsystem;
		addRequirements(m_intakeArmSubsystem);
	}

	@Override
	public void execute() {
		// debugLogger.logln("" + driveSubsystem.getPos().getY());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
	}

}
