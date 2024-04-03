

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FlywheelSubsystem;


public class FlywheelCalibrate extends Command {
	private FlywheelSubsystem m_flywheelSubsystem;

	public FlywheelCalibrate(FlywheelSubsystem flywheelSubsystem) {
		m_flywheelSubsystem = flywheelSubsystem;
		addRequirements(flywheelSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.setSpeed(-0.3);
        m_flywheelSubsystem.resetSholderEncoder();
	}

	@Override
	public boolean isFinished() {
        return false;
	}
	@Override
    public void end(boolean interrupted){
        m_flywheelSubsystem.setSpeed(0.0);
    }    
}


