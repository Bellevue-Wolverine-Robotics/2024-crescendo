package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class GetIntakeStatusFlywheel extends Command {
    public FlywheelSubsystem flywheel;
    public GetIntakeStatusFlywheel(FlywheelSubsystem flywheelSubsystem){
        flywheel = flywheelSubsystem;
    }
    @Override
    public boolean isFinished(){
        return flywheel.hasNote();
    }
    public void end(){
        
    }
}
