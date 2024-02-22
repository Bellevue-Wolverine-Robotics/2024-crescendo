package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetArmAngleCommand extends Command {
	private IntakeSubsystem m_intakeArmSubsystem;
	private double m_setpoint;

	public SetArmAngleCommand(IntakeSubsystem intakeArmSubsystem, double setpoint) {
		m_intakeArmSubsystem = intakeArmSubsystem;
		m_setpoint = setpoint;

		addRequirements(m_intakeArmSubsystem);
	}

	@Override
	public void initialize() {
		m_intakeArmSubsystem.setIntakeArmPIDSetpoint(m_setpoint);
	}

}
