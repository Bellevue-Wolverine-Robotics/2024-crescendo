package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Debug;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PIDUtils;

public class DriveStraight extends Command {
    double kPStraight = 2.0;
    double kIStraight = 0.0;
    double kDStraight = 0.0;

    private PIDController m_pidLinear = new PIDController(kPStraight, kIStraight, kDStraight);

    private DriveSubsystem m_driveSubsystem;
    private double m_targetDistance;
    public DriveStraight(DriveSubsystem driveSubsystem, double distance){
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = distance; 

        m_pidLinear.setTolerance(0.1);

        addRequirements(m_driveSubsystem);
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