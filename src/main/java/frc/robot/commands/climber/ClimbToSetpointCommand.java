package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbToSetpointCommand extends Command {
    private ClimberSubsystem m_climberSubsystem;

    private double m_setpoint;

    public ClimbToSetpointCommand(ClimberSubsystem climberSubsystem, double setpoint) {
        this.m_climberSubsystem = climberSubsystem;
        this.m_setpoint = setpoint;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setClimbPIDSetpoint(m_setpoint);
    }

}
