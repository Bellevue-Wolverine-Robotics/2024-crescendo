package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Debug;

public class DebugClose extends Command{
    public DebugClose(Debug debugLogger){
        debugLogger.closelog();
    }
    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){}
    
}
