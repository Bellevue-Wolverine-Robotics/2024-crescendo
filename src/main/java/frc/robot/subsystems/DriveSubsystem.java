package frc.robot.subsystems;

import java.lang.Object;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Throttles;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax m_leftBack = new CANSparkMax(CANConstants.backLeft, MotorType.kBrushless);
    private CANSparkMax m_leftFront = new CANSparkMax(CANConstants.frontLeft, MotorType.kBrushless);
    private CANSparkMax m_rightFront = new CANSparkMax(CANConstants.frontRight, MotorType.kBrushless);
    private CANSparkMax m_rightBack = new CANSparkMax(CANConstants.backRight, MotorType.kBrushless);

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

    private RelativeEncoder m_leftEncoder = m_leftFront.getEncoder();
    private RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();
    private DifferentialDriveOdometry m_odometry;

    private AHRS m_imu = new AHRS(SPI.Port.kMXP);

    public DriveSubsystem() {
        m_leftBack.follow(m_leftFront);
        m_rightBack.follow(m_rightFront);

        this.m_leftFront.restoreFactoryDefaults();
        this.m_leftBack.restoreFactoryDefaults();
        this.m_rightFront.restoreFactoryDefaults();
        this.m_rightBack.restoreFactoryDefaults();

        m_leftFront.setSmartCurrentLimit(30);
        m_rightFront.setSmartCurrentLimit(30);
        m_leftBack.setSmartCurrentLimit(30);
        m_rightBack.setSmartCurrentLimit(30);

        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftBack.setIdleMode(IdleMode.kCoast);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightBack.setIdleMode(IdleMode.kCoast);

        m_leftFront.setInverted(true);
        /* Only voltage output is mirrored. Settings changed on the leader do not affect the follower. */
        /*The motor will spin in the same direction as the leader. This can be changed by passing a true constant after the leader parameter. */
    


        m_odometry = new DifferentialDriveOdometry(
                m_imu.getRotation2d(),
                m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                new Pose2d());
        m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                new Pose2d());

    }

    public Pose2d getPos() {
        return m_odometry.getPoseMeters();
    }

    public void tankDrive(double left, double right) {
        this.m_drive.tankDrive(left * Throttles.limit, right * Throttles.limit);
    }

    public void arcadeDrive(double xSpeed, double rotation) {
        this.m_drive.arcadeDrive(xSpeed * Throttles.limit, rotation * Throttles.limit);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        System.out.println("left: " + m_leftFront.getOutputCurrent());
        System.out.println("right: " + m_rightFront.getOutputCurrent());

    }

}
