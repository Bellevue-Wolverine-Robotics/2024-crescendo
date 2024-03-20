package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;


public class FlywheelAuxShootUntilStop extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelAuxShootUntilStop(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.startShooter();
        m_flywheelSubsystem.startFeeder();
	}

	@Override
	public boolean isFinished() {
        return false;
	}
	@Override
    public void end(boolean interrupted){
        m_flywheelSubsystem.stopShooter();
        m_flywheelSubsystem.stopFeeder();

    }    
}


