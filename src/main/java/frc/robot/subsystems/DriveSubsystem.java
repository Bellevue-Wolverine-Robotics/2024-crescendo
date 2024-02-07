package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants;
import frc.robot.Debug;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

    private SparkPIDController m_leftPID;
    private SparkPIDController m_rightPID;

    private double speedLimit;
    private AHRS m_imu = new AHRS(SPI.Port.kMXP);
    private Debug debugLogger;

    private Field2d m_field = new Field2d();

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

        BooleanSupplier bsupply = (() -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        });



        m_leftPID = m_leftFront.getPIDController();
        m_rightPID = m_rightFront.getPIDController();

        buildPidController(m_leftPID);
        buildPidController(m_rightPID);

                AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                bsupply::getAsBoolean, // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                this); // Reference to this subsystem to set requirements
                        System.out.println("t commaqngfdhjilhukgyftghkjhgf");

    }


    // path planner
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.635);

        // Example differential drive wheel speeds: 2 meters per second
        // for the left side, 3 meters per second for the right side.
        var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());

        // Convert to chassis speeds.
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);


        return chassisSpeeds;

        // return (m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity()) / 2;
    }

    public void resetPose() {
        this.resetPose(this.getPose());
    }

    public void tankDrive(double left, double right) {
        this.m_drive.tankDrive(left * speedLimit, right * speedLimit);
    }

    public void drive2(ChassisSpeeds speeds) {

        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
                Constants.DriveConstants.trackWidthMeters);
        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        // Left velocity
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;

        // Right velocity
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        this.m_drive.tankDrive(leftVelocity * speedLimit, rightVelocity * speedLimit);
    }



    private void buildPidController(SparkPIDController pidController) {
        pidController.setP(1e-6);
        pidController.setI(ClimbingConstants.kI);
        pidController.setD(ClimbingConstants.kD);
        pidController.setIZone(ClimbingConstants.kIZone);
        pidController.setFF(ClimbingConstants.kFF);
        pidController.setOutputRange(ClimbingConstants.kMinOutput, ClimbingConstants.kMaxOutput);
    }

    public void drive(ChassisSpeeds speed){
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
                Constants.DriveConstants.trackWidthMeters);
        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);

        // Left velocity
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;

        // Right velocity
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        double encoderLeft = leftVelocity/((DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);
        double encoderRight = rightVelocity/((DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);


        m_leftPID.setReference(encoderLeft, ControlType.kVelocity);
        m_rightPID.setReference(encoderRight, ControlType.kVelocity);

    }

    public Command testCommand(){
        System.out.println("t commaqngfdhjilhukgyftghkjhgf");
         double leftVelocity = 4.0;

        // Right velocity
        double rightVelocity = 4.0;

        double encoderLeft = leftVelocity/((DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);
        double encoderRight = rightVelocity/((DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.DRIVE_GEAR_RATIO) / 60);
        
        return this.runOnce(()->{
            m_leftPID.setReference(encoderLeft, ControlType.kVelocity);
            m_rightPID.setReference(encoderRight, ControlType.kVelocity);
        });
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
        SmartDashboard.putNumber("Current Y position: ", getPose().getY());
        SmartDashboard.putNumber("Current X position: ", getPose().getX());
        SmartDashboard.putNumber("Current Heading: ", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("m_imu: ", m_imu.getAngle());

        SmartDashboard.putNumber("Left Encoder: ", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder: ", m_rightEncoder.getPosition());

        m_field.setRobotPose(m_odometry.getPoseMeters());
        //System.out.println("Odometry Pos: X:" + getPose().getX() + "Y: " + getPose().getY());

    }

}
