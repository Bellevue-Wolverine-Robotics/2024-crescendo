package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EmpiricalConstants;
import frc.robot.Debug;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax m_leftBack = new CANSparkMax(DriveConstants.backLeftId, MotorType.kBrushless);
    private CANSparkMax m_leftFront = new CANSparkMax(DriveConstants.frontLeftId, MotorType.kBrushless);
    private CANSparkMax m_rightFront = new CANSparkMax(DriveConstants.frontRightId, MotorType.kBrushless);
    private CANSparkMax m_rightBack = new CANSparkMax(DriveConstants.backRightId, MotorType.kBrushless);

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

    private RelativeEncoder m_leftEncoder = m_leftFront.getEncoder();
    private RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();

    private DifferentialDriveOdometry m_odometry;

    private SparkPIDController m_frontLeftPID;
    private SparkPIDController m_frontRightPID;
    private SparkPIDController m_backLeftPID;
    private SparkPIDController m_backRightPID;

    private AHRS m_imu = new AHRS(SPI.Port.kMXP);
    private Debug debugLogger;

    private Field2d m_field = new Field2d();
    // private VisionSubsystem vision = new VisionSubsystem();// NESTED
    // SUBSYSTEMS????
    // private Pose2d currPose2d;

    private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(0.0),
            m_imu.getRotation2d(),
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // TODO: update these to reflect actual values using SysId
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
            7.29, // 7.29:1 gearing reduction.
            7.5, // MOI of 7.5 kg m^2 (from CAD model).
            60.0, // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.7112, // The track width is 0.7112 meters.

            // The standard deviations for measurement noise:
            // x and y: 0.001 m
            // heading: 0.001 rad
            // l and r velocity: 0.1 m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

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

        m_rightFront.setInverted(true);

        m_imu.reset();
        m_imu.resetDisplacement();
        m_imu.zeroYaw();

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

        m_frontLeftPID = m_leftFront.getPIDController();
        m_frontRightPID = m_rightFront.getPIDController();
        m_backLeftPID = m_leftBack.getPIDController();
        m_backRightPID = m_rightBack.getPIDController();

        buildPidController(m_frontLeftPID);
        buildPidController(m_frontRightPID);
        buildPidController(m_backLeftPID);
        buildPidController(m_backRightPID);

        AutoBuilder.configureRamsete(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                this::drive,
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                bsupply::getAsBoolean, // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                this);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidthMeters);

        var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        return chassisSpeeds;
    }

    public void tankDrive(double left, double right) {
        this.m_drive.tankDrive(left, right);
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
                Constants.DriveConstants.trackWidthMeters);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        // TODO: TEST IF WE NEED TO APPLY THESE TO THE BACK MOTORCONTROLLER PIDS TOO
        m_frontLeftPID.setReference(leftVelocity, ControlType.kVelocity);
        m_frontRightPID.setReference(rightVelocity, ControlType.kVelocity);

        m_drive.feed();
    }

    private void buildPidController(SparkPIDController pidController) {
        pidController.setP(DriveConstants.kP);
        pidController.setI(DriveConstants.kI);
        pidController.setD(DriveConstants.kD);
        pidController.setIZone(DriveConstants.kIZone);
        pidController.setFF(DriveConstants.kFF);
        pidController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
    }

    public void arcadeDrive(double xSpeed, double rotation) {
        this.m_drive.arcadeDrive(xSpeed, rotation);
    }

    public double getYaw() {
        return m_imu.getYaw();
    }

    public void stopDriveTrain() {
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {
        // currPose2d = m_poseEstimator.update(m_imu.getRotation2d(),
        // m_leftEncoder.getPosition(),
        // m_rightEncoder.getPosition());
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

        SmartDashboard.putNumber("Front Left Duty Cycle: ", m_leftFront.get());
        SmartDashboard.putNumber("Front Left Distance: ", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Front Left Rate: ", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Back Left Rate: ", m_leftBack.getEncoder().getVelocity());

        SmartDashboard.putNumber("Sim Y position: ", m_driveSim.getPose().getY());
        SmartDashboard.putNumber("Sim X position: ", m_driveSim.getPose().getX());

        m_field.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInputs(m_leftFront.get() * EmpiricalConstants.kInputVoltage,
                m_rightFront.get() * EmpiricalConstants.kInputVoltage);

        System.out.println(m_leftFront.get());

        m_driveSim.update(0.02);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-m_driveSim.getHeading().getDegrees());
    }

    /**
     * Returns CanSparkMax array in format of
     * [frontLeft, frontRight, backLeft, backRight]
     */
    public CANSparkMax[] getDriveMotorControllers() {
        return new CANSparkMax[] { m_leftFront, m_rightFront, m_leftBack, m_rightBack };

        // andrew i was trying to resolve a merge conflict here idk what this code does

        // vision.getEstimatedGlobalPose(getPose());
        // m_poseEstimator.addVisionMeasurement(vision.getPose2d(),
        // vision.getTimestampSeconds());
    }

}