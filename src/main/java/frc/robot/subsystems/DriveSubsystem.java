package frc.robot.subsystems;

import java.lang.Object;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.Throttles;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase{
    private CANSparkMax m_leftBack = new CANSparkMax(CANConstants.backLeft, MotorType.kBrushless);
    private CANSparkMax m_leftFront = new CANSparkMax(CANConstants.frontLeft, MotorType.kBrushless);
    private CANSparkMax m_rightFront = new CANSparkMax(CANConstants.frontLeft, MotorType.kBrushless);
    private CANSparkMax m_rightBack = new CANSparkMax(CANConstants.backRight, MotorType.kBrushless);

    private MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftBack);
    private MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightBack);

    private DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    public DriveSubsystem() {
        this.m_leftFront.restoreFactoryDefaults();
        this.m_leftBack.restoreFactoryDefaults();
        this.m_rightFront.restoreFactoryDefaults();
        this.m_rightBack.restoreFactoryDefaults();

        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftBack.setIdleMode(IdleMode.kBrake);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightBack.setIdleMode(IdleMode.kBrake);

        m_rightFront.setInverted(true);
        m_rightBack.setInverted(true);
    

    }

    public Pose2d getPos(){
        //TODO: stop retrun dummy
        return new Pose2d();
    }
    public void tankDrive(double left, double right){
        this.m_drive.tankDrive(left * Throttles.limit, right * Throttles.limit);
    }
    public void arcadeDrive(double xSpeed, double rotation){
        this.m_drive.arcadeDrive(xSpeed * Throttles.limit, rotation * Throttles.limit);
    }
}
