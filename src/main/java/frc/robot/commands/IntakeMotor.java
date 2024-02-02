package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeMotorSubsystem;

public class IntakeMotor extends Command {
    public IntakeMotorSubsystem m_intakeMotorSubsystem;

    public IntakeMotor(IntakeMotorSubsystem intakeMotorSubsystem) {
        this.m_intakeMotorSubsystem = intakeMotorSubsystem;
        addRequirements(this.m_intakeMotorSubsystem);

    }

    @Override
    public void initialize() {
        m_intakeMotorSubsystem.setIntakeMotor(IntakeConstants.intakeMotorSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_intakeMotorSubsystem.currentlyAcquired();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeMotorSubsystem.setIntakeMotor(0.0);
    }

}
