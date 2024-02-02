package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Debug;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax m_leftBack = new CANSparkMax(DriveConstants.backLeftId, MotorType.kBrushless);
    private CANSparkMax m_leftFront = new CANSparkMax(DriveConstants.frontLeftId, MotorType.kBrushless);
    private CANSparkMax m_rightFront = new CANSparkMax(DriveConstants.frontRightId, MotorType.kBrushless);
    private CANSparkMax m_rightBack = new CANSparkMax(DriveConstants.backRightId, MotorType.kBrushless);

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

    private RelativeEncoder m_leftEncoder = m_leftFront.getEncoder();
    private RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();
    private DifferentialDriveOdometry m_odometry;

    private double speedLimit;
    private AHRS m_imu = new AHRS(SPI.Port.kMXP);
    private Debug debugLogger;

    private Field2d m_field = new Field2d();
    private VisionSubsystem vision = new VisionSubsystem();//NESTED SUBSYSTEMS????



    private final DifferentialDrivePoseEstimator m_poseEstimator =
    new DifferentialDrivePoseEstimator(
        new DifferentialDriveKinematics(0.0),
        m_imu.getRotation2d(),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));



    public DriveSubsystem(Debug debugLogger) {
        SmartDashboard.putData("Field", m_field);

        this.m_leftFront.restoreFactoryDefaults();
        this.m_leftBack.restoreFactoryDefaults();
        this.m_rightFront.restoreFactoryDefaults();
        this.m_rightBack.restoreFactoryDefaults();

        m_leftFront.setSmartCurrentLimit(30);
        m_rightFront.setSmartCurrentLimit(30);
        m_leftBack.setSmartCurrentLimit(30);
        m_rightBack.setSmartCurrentLimit(30);

        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftBack.setIdleMode(IdleMode.kBrake);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightBack.setIdleMode(IdleMode.kBrake);

        m_leftBack.follow(m_leftFront);
        m_rightBack.follow(m_rightFront);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        m_leftEncoder.setPositionConversionFactor(
                DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO);
        m_rightEncoder.setPositionConversionFactor(
                DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO);

        // WPILIB expects encoder rate to be in M/S while REV returns M/Min
        m_leftEncoder.setVelocityConversionFactor(
                (DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);
        m_rightEncoder.setVelocityConversionFactor(
                (DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);

        m_leftFront.setInverted(true);

        m_imu.reset();
        m_imu.resetDisplacement();
        m_imu.zeroYaw();

        speedLimit = DriveConstants.limit;
        /*
         * Only voltage output is mirrored. Settings changed on the leader do not affect
         * the follower.
         */
        /*
         * The motor will spin in the same direction as the leader. This can be changed
         * by passing a true constant after the leader parameter.
         */

        m_odometry = new DifferentialDriveOdometry(
                m_imu.getRotation2d(),
                m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                new Pose2d());
        m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                new Pose2d());

        this.debugLogger = debugLogger;

    }

    public Pose2d getPos() {
        return m_odometry.getPoseMeters();
    }

    public void tankDrive(double left, double right) {
        this.m_drive.tankDrive(left * speedLimit, right * speedLimit);
    }

    public void arcadeDrive(double xSpeed, double rotation) {
        this.m_drive.arcadeDrive(xSpeed * speedLimit, rotation * speedLimit);
    }

    public void setSpeedLimit(double speedLimit) {
        this.speedLimit = speedLimit;
    }

    public double getSpeedLimit() {
        return this.speedLimit;
    }

    @Override
    public void periodic() {
        m_odometry.update(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        /*
         * debugLogger.logln("leftFront: " + m_leftFront.getOutputCurrent() +
         * " rightFront: " + m_rightFront.getOutputCurrent()
         * + "      |||      leftBack: " + m_leftBack.getOutputCurrent() +
         * "  rightBack: " + m_rightBack.getOutputCurrent());
         */
        SmartDashboard.putNumber("Current Y position: ", getPos().getY());
        SmartDashboard.putNumber("Current X position: ", getPos().getX());
        SmartDashboard.putNumber("Current Heading: ", getPos().getRotation().getDegrees());
        SmartDashboard.putNumber("m_imu: ", m_imu.getAngle());

        SmartDashboard.putNumber("Left Encoder: ", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder: ", m_rightEncoder.getPosition());

        m_field.setRobotPose(m_odometry.getPoseMeters());
        vision.getEstimatedGlobalPose(getPos());
        m_poseEstimator.addVisionMeasurement(vision.getPose2d(), vision.getTimestampSeconds());
    }
    


    
}