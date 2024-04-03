package frc.robot.commands.climber;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;


public class ClimberCalibrate extends Command {
	private ClimberSubsystem m_ClimberSubsystem;

    //climberSubsystem
	public ClimberCalibrate(ClimberSubsystem climberSubsystem) {
		m_ClimberSubsystem = climberSubsystem;
		addRequirements(m_ClimberSubsystem);
	}

	@Override
	public void initialize() {
		m_ClimberSubsystem.setSpeed(-0.3);
	}

	@Override
	public boolean isFinished() {
        return false;
	}
	@Override
    public void end(boolean interrupted){
        m_ClimberSubsystem.setSpeed(0.0);
    }    
}


