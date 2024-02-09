package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Debug;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends Command{
    private PIDController m_pidLinear = new PIDController(DriveConstants.kPStraight, DriveConstants.kIStraight, DriveConstants.kDStraight);

    private DriveSubsystem m_driveSubsystem;

    private double m_targetDistance;
    DriveSubsystem driveSubsystem;
    Debug debugLogger;

    public DriveStraight(DriveSubsystem driveSubsystem, Debug debugLogger, double distance){
        driveSubsystem.resetPose();
        this.driveSubsystem = driveSubsystem;
        this.debugLogger = debugLogger;
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = -distance; 

        m_pidLinear.setTolerance(0.1);
        addRequirements(driveSubsystem);
    }

    public DriveStraight(DriveSubsystem driveSubsystem, double distance){

    }

    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getX(), m_targetDistance), -0.7, 0.7);

        m_driveSubsystem.tankDrive(linearSpeed, linearSpeed);
    }

    @Override
    public boolean isFinished() {

        return m_pidLinear.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.stopDriveTrain();
    }

    
}
