package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Debug;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PIDUtils;

/*
 * 
 * 
 * 
 * 
 * DifferentialDrive... Output not updated often enough. See https://docs.wpilib.org/motorsafety for more information.
   from: edu.wpi.first.wpilibj.MotorSafety.check(MotorSafety.java:140)


 */
public class DriveStraight extends Command {
    private PIDController m_pidLinear = PIDUtils.createPIDController(DriveConstants.kStraightPidParams);
    private DriveSubsystem m_driveSubsystem;

    private double m_targetDistance;
    DriveSubsystem driveSubsystem;
    Debug debugLogger;

    public DriveStraight(DriveSubsystem driveSubsystem, Debug debugLogger, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.debugLogger = debugLogger;
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = -distance;

        m_pidLinear.setTolerance(0.1);
        addRequirements(driveSubsystem);
    }

    public DriveStraight(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = -distance;

        m_pidLinear.setTolerance(0.1);
        addRequirements(driveSubsystem);

    }

    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getX(), m_targetDistance),
                -0.7, 0.7);

        m_driveSubsystem.tankDrive(linearSpeed, linearSpeed);
    }

    @Override
    public boolean isFinished() {

        return m_pidLinear.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stopDriveTrain();
    }

}
