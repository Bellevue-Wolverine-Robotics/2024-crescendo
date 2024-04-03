package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.DebugSettings;
import frc.robot.constants.FlywheelConstants;
import frc.robot.constants.IntakeConstants;
import frc.utils.PIDUtils;

public class FlywheelSubsystem extends SubsystemBase {
	private WPI_TalonSRX m_shooterMotorLeader;
	private WPI_TalonSRX m_shooterMotorFollower;

	private CANSparkMax m_armShoulderMotor;
	private CANSparkMax m_armElbowMotor;

	private RelativeEncoder m_armShoulderEncoder;
	private RelativeEncoder m_armElbowEncoder;

	private SparkPIDController m_armShoulderPidController;
	private SparkPIDController m_armElbowPidController;

	private TalonSRX m_feederMotor; // this is a motor that collects the note from the intake

	private DigitalInput m_noteLimitSwitch = new DigitalInput(FlywheelConstants.kNoteSwitchDIOPort);

	public FlywheelSubsystem() {
		// Shooter Init
		m_shooterMotorLeader = new WPI_TalonSRX(FlywheelConstants.kShooterLeaderId);
		m_shooterMotorFollower = new WPI_TalonSRX(FlywheelConstants.kShooterFollowerId);
		m_feederMotor = new WPI_TalonSRX(FlywheelConstants.kFeederId);

		// Arm Init
		m_armShoulderMotor = new CANSparkMax(FlywheelConstants.kArmShoulderId, MotorType.kBrushless);
		m_armElbowMotor = new CANSparkMax(FlywheelConstants.kArmElbowId, MotorType.kBrushless);

		m_armShoulderMotor.restoreFactoryDefaults();
		m_armElbowMotor.restoreFactoryDefaults();

		m_shooterMotorLeader.configFactoryDefault();
		m_shooterMotorFollower.configFactoryDefault();
		m_feederMotor.configFactoryDefault();
		m_feederMotor.setNeutralMode(NeutralMode.Coast);


		m_shooterMotorFollower.follow(m_shooterMotorLeader);
		m_shooterMotorFollower.setInverted(true);

		m_armShoulderEncoder = m_armShoulderMotor.getEncoder();
		m_armElbowEncoder = m_armElbowMotor.getEncoder();

		m_armShoulderEncoder.setPosition(0);
		m_armElbowEncoder.setPosition(0);

		m_armShoulderPidController = m_armShoulderMotor.getPIDController();
		m_armElbowPidController = m_armElbowMotor.getPIDController();

		m_armShoulderMotor.setIdleMode(IdleMode.kBrake);
		m_armElbowMotor.setIdleMode(IdleMode.kBrake);

		PIDUtils.setPIDConstants(m_armShoulderPidController, FlywheelConstants.kArmShoulderPid);
		PIDUtils.setPIDConstants(m_armElbowPidController, FlywheelConstants.kArmElbowPid);

		SmartDashboard.putNumber("m_armShoulderPidController kP", m_armShoulderPidController.getP());
		SmartDashboard.putNumber("m_armShoulderPidController kI", m_armShoulderPidController.getI());
		SmartDashboard.putNumber("m_armShoulderPidController kD", m_armShoulderPidController.getD());
		SmartDashboard.putNumber("m_armShoulderPidController kff", m_armShoulderPidController.getFF());

		SmartDashboard.putNumber("m_armElbowPidController kP", m_armElbowPidController.getP());
		SmartDashboard.putNumber("m_armElbowPidController kI", m_armElbowPidController.getI());
		SmartDashboard.putNumber("m_armElbowPidController kD", m_armElbowPidController.getD());
		SmartDashboard.putNumber("m_armElbowPidController kff", m_armElbowPidController.getFF());
	}

	public void setElbow(double dutyCycle) {
		m_armElbowMotor.set(dutyCycle);
	}

	public void stopElbowMotor() {
		m_armElbowMotor.stopMotor();
	}


	public void calibrateRetract(){
		System.out.println("here callibrating");
		m_armShoulderMotor.setVoltage(-1);
		m_armShoulderEncoder.setPosition(0.0);
	}

	public void setSpeed(double speed){
		m_armShoulderMotor.set(speed);
	}

	public void resetSholderEncoder(){
		m_armShoulderEncoder.setPosition(0.0);
	}


	public void setShooterDutyCycle(double dutyCycle) {
		m_shooterMotorLeader.set(TalonSRXControlMode.PercentOutput, dutyCycle);
	}

	public void startShooter() {
		setShooterDutyCycle(FlywheelConstants.kShootSpeakerDutyCycleSetpoint);
	}

	public void stopShooter() {
		setShooterDutyCycle(0);
	}

	public void setArmSetpoint(double shoulderSetpoint, double elbowSetpoint) {
		m_armShoulderPidController.setReference(shoulderSetpoint, ControlType.kPosition);
		m_armElbowPidController.setReference(elbowSetpoint, ControlType.kPosition);
	}

	public void aimArmToSpeaker() {
		setArmSetpoint(FlywheelConstants.kSpeakerShoulderSetpoint, FlywheelConstants.kSpeakerElbowSetpoint);
	}

	public void aimArmToMakeSpaceForIntake() {
		setArmSetpoint(FlywheelConstants.kIntakeMakeSpaceShoulderSetpoint,
				FlywheelConstants.kIntakeMakeSpaceElbowSetpoint);
	}


	public void aimLowerPosition(){
		setArmSetpoint(FlywheelConstants.kIntakeShooterShoulderSetpoint,
				FlywheelConstants.kIntakeShooterElbowSetpoint);
	}


	public void setShoulderAimSpeaker(){
		m_armShoulderPidController.setReference(FlywheelConstants.kSpeakerShoulderSetpoint, ControlType.kPosition);
	}

	public void setElbowAimSpeaker(){
		m_armElbowPidController.setReference(FlywheelConstants.kSpeakerElbowSetpoint, ControlType.kPosition);
	}

	public void aimArmToAmp() {
		setArmSetpoint(FlywheelConstants.kAmpShoulderSetpoint, FlywheelConstants.kAmpElbowSetpoint);
	}


	public void climbMode() {
		setArmSetpoint(FlywheelConstants.kIntakeClimbShoulderSetpoint, FlywheelConstants.kIntakeClimbElbowSetpoint);
	}
	/*
	 * This method aims the arm to receive the note from the intake
	 */
	public void aimArmToIntake() {
		setArmSetpoint(FlywheelConstants.kIntakeReceiveShoulderSetpoint, FlywheelConstants.kIntakeReceiveElbowSetpoint);
	}

	public boolean isArmAimingTowardsIntake() {
		return isArmAtSetpoint(FlywheelConstants.kIntakeReceiveShoulderSetpoint,
				FlywheelConstants.kIntakeReceiveElbowSetpoint);
	}

	public boolean isArmClimbMode() {
		return isArmAtSetpoint(FlywheelConstants.kIntakeClimbShoulderSetpoint,
				FlywheelConstants.kIntakeClimbElbowSetpoint);
	}

	public boolean isArmAimingTowardsAmp() {
		return isArmAtSetpoint(FlywheelConstants.kAmpShoulderSetpoint, FlywheelConstants.kAmpElbowSetpoint);
	}

	public boolean isArmAimingTowardsSpeaker() {
		return isArmAtSetpoint(FlywheelConstants.kSpeakerShoulderSetpoint, FlywheelConstants.kSpeakerElbowSetpoint);
	}

	public boolean isArmMakingSpaceForIntake() {
		return isArmAtSetpoint(FlywheelConstants.kIntakeMakeSpaceShoulderSetpoint,
				FlywheelConstants.kIntakeMakeSpaceElbowSetpoint);
	}

	public boolean isArmAtSetpoint(double shoulderSetpoint, double elbowSetpoint) {
		return PIDUtils.atSetpoint(m_armShoulderMotor.getEncoder().getPosition(), shoulderSetpoint,
				FlywheelConstants.kArmShoulderTolerance) &&
				PIDUtils.atSetpoint(m_armElbowMotor.getEncoder().getPosition(), elbowSetpoint,
						FlywheelConstants.kArmElbowTolerance);
	}

	//public int testingstartFeederDeleteMe = 1;
	public void startFeeder() {

		//System.out.println("startfeeder count: " + testingstartFeederDeleteMe);
		//testingstartFeederDeleteMe ++ ;
		m_feederMotor.set(TalonSRXControlMode.PercentOutput, FlywheelConstants.kFeederDutyCycleSetpoint);
	}
	public void startFeederReverse() {
		m_feederMotor.set(TalonSRXControlMode.PercentOutput, FlywheelConstants.kFeederDutyCycleSetpoint);
	}


	//public int stopFeederDELETEME = 1;
	public void stopFeeder() {
		//System.out.println("stopping feeder motor: FLywheelsubsystem: " + stopFeederDELETEME);
		//stopFeederDELETEME++;
		m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
	}

	@Override
	public void periodic() {
		if(DebugSettings.debugMode){
			var armShoulderParams = new PIDUtils.SparkPIDParams(m_armShoulderMotor);
			var armElbowParams = new PIDUtils.SparkPIDParams(m_armElbowMotor);

			armShoulderParams.changeKp(SmartDashboard.getNumber("m_armShoulderPidController kP", m_armShoulderPidController.getP()));
			armShoulderParams.changeKi(SmartDashboard.getNumber("m_armShoulderPidController kI", m_armShoulderPidController.getI()));
			armShoulderParams.changeKd(SmartDashboard.getNumber("m_armShoulderPidController kD", m_armShoulderPidController.getD()));
			armShoulderParams.changeKff(SmartDashboard.getNumber("m_armShoulderPidController kff", m_armShoulderPidController.getFF()));

			armShoulderParams.changeKp(SmartDashboard.getNumber("m_armElbowPidController kP", m_armElbowPidController.getP()));
			armShoulderParams.changeKi(SmartDashboard.getNumber("m_armElbowPidController kI", m_armElbowPidController.getI()));
			armShoulderParams.changeKd(SmartDashboard.getNumber("m_armElbowPidController kD", m_armElbowPidController.getD()));
			armShoulderParams.changeKff(SmartDashboard.getNumber("m_armElbowPidController kff", m_armElbowPidController.getFF()));

			PIDUtils.setPIDConstants(m_armShoulderPidController, armShoulderParams);
			PIDUtils.setPIDConstants(m_armElbowPidController, armElbowParams);
		}

		SmartDashboard.putNumber("SHOULDER ANGLE", m_armShoulderEncoder.getPosition());
		SmartDashboard.putNumber("ELBOW ANGLE", m_armElbowEncoder.getPosition());
	}

	public boolean hasNote() {
		//System.out.println("limit switch status" + m_noteLimitSwitch.get());
		return m_noteLimitSwitch.get();
	}

	public void debugFeeder(CommandJoystick joystick) {
		m_feederMotor.set(TalonSRXControlMode.PercentOutput, -joystick.getY());
	}

	public void debugShooter(CommandJoystick joystick) {
		m_shooterMotorLeader.set(TalonSRXControlMode.PercentOutput, -joystick.getY());
	}

	public void debugElbow(CommandJoystick joystick) {
		m_armElbowMotor.set(-joystick.getY() / 10);
	}

	public void debugShoulder(CommandJoystick joystick) {
		m_armShoulderMotor.set(-joystick.getY() / 10);
	}

	public double getShoulderEncoderValue() {
		return m_armShoulderEncoder.getPosition();
	}

	public double getElbowEncoderValue() {
		return m_armElbowEncoder.getPosition();
	}

}
