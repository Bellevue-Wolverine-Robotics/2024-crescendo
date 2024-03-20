package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;


public class FlywheelMainShootUntilStop extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelMainShootUntilStop(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.startShooter();
	}

	@Override
	public boolean isFinished() {
        return false;
	}
	@Override
    public void end(boolean interrupted){
        m_flywheelSubsystem.stopShooter();
    }    
}


