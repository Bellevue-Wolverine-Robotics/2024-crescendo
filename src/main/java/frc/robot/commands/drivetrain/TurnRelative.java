package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PIDUtils;

public class TurnRelative extends Command {
    double kPTurn = 2.0;
    double kIurn = 0.0;
    double kDturn = 0.0;

    private PIDController m_pidLinear = new PIDController(kPTurn, kIurn, kDturn);

    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    public TurnRelative(DriveSubsystem driveSubsystem, double angle){ //ANGLE IN DEGREES
        m_driveSubsystem = driveSubsystem;
        m_targetAngle = angle; 

        m_pidLinear.setTolerance(5);
        addRequirements(m_driveSubsystem);
    }


    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getRotation().getDegrees(), m_targetAngle), -0.7, 0.7);
        m_driveSubsystem.tankDrive(linearSpeed, -linearSpeed);
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