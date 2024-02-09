package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Debug;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends Command{
    DriveSubsystem driveSubsystem;
    Debug debugLogger;

    public DriveStraight(DriveSubsystem driveSubsystem, Debug debugLogger){
        this.driveSubsystem = driveSubsystem;
        this.debugLogger = debugLogger;
        addRequirements(driveSubsystem);
    }

    public DriveStraight(DriveSubsystem driveSubsystem, double distance){

    }

    @Override
    public void execute(){
        driveSubsystem.tankDrive(0.5, 0.5);
        //debugLogger.logln("" + driveSubsystem.getPos().getY());
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.tankDrive(0.0, 0.0);
    }

    
}
