package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedNoteFromIntakeToFlywheelCommand extends Command {
	private IntakeSubsystem m_intakeSubsystem;
	private FlywheelSubsystem m_flywheelSubsystem;

	public FeedNoteFromIntakeToFlywheelCommand(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;

		addRequirements(m_intakeSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.startFeedIntoFlywheel();
		m_flywheelSubsystem.startFeeder();
	}

	@Override
	public boolean isFinished() {
		//if(m_flywheelSubsystem.hasNote())System.out.println("Testing termination " + m_flywheelSubsystem.hasNote());
		return m_flywheelSubsystem.hasNote();
	}

	@Override
	public void end(boolean interrupted) {
		//System.out.println("Interupted: " + m_flywheelSubsystem.hasNote());
		
		m_intakeSubsystem.stopIntakeMotor();
		m_flywheelSubsystem.stopFeeder();
	}
}
