package frc.robot.constants;

import frc.utils.PIDUtils;

public final class FlywheelConstants {
	public static final double kShooterMaxRPM = 5828; // used for calculating FF

	// CAN IDs
	public static final int kArmShoulderId = 9;
	public static final int kArmElbowId = 10;
	public static final int kShooterLeaderId = 11;
	public static final int kShooterFollowerId = 12;
	public static final int kFeederId = 13;

	// PID

	public static final PIDUtils.TalonPIDParams kShooterVelocityPid = new PIDUtils.TalonPIDParams(6e-4, 0, 0, 0,
			MotorConstants.kTalonClosedLoopFullOutput / kShooterMaxRPM);

	public static final PIDUtils.SparkPIDParams kArmShoulderPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kArmElbowPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kFeederPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);

	// Setpoints

	public static final double kShootStageDutyCycleSetpoint = 0.5;

	public static final double kStageShoulderSetpoint = 0;
	public static final double kStageElbowSetpoint = 0;

	public static final double kAmpShoulderSetpoint = 0;
	public static final double kAmpElbowSetpoint = 00;

	public static final double kFeederRelativeSetpoint = 0; // how far the feeder motor should travel relative to its
															// current position when feeding into the shooter
}
