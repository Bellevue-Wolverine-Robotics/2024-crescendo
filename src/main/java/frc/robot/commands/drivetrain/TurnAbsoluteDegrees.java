// TODO: considering removing this command + DriveStraight.java because we can just use PathPlanner

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnAbsoluteDegrees extends Command {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn,
            DriveConstants.kDTurn);

    public TurnAbsoluteDegrees(double absoluteDegrees, DriveSubsystem driveSubsystem) {
        absoluteDegrees *= 0.9;

        m_driveSubsystem = driveSubsystem;
        m_targetAngle = absoluteDegrees;

        m_pid.setTolerance(5);
        m_pid.enableContinuousInput(-180, 180);

        addRequirements(m_driveSubsystem);

    }

    public void execute() {
        double motorSpeed = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getYaw(), m_targetAngle), 0, 0.8);

        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

}
