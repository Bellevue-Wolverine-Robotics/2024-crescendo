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

	public static final PIDUtils.SparkPIDParams kArmShoulderPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kArmElbowPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kFeederPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);

	// Setpoints

	public static final double kArmShoulderTolerance = 1;
	public static final double kArmElbowTolerance = 1;

	public static final double kShootSpeakerDutyCycleSetpoint = 0.5;
	public static final double kFeederDutyCycleSetpoint = 0.8;

	public static final double kSpeakerShoulderSetpoint = 0;
	public static final double kSpeakerElbowSetpoint = 0;

	public static final double kAmpShoulderSetpoint = 0;
	public static final double kAmpElbowSetpoint = 0;

	public static final double kIntakeReceiveShoulderSetpoint = 0;
	public static final double kIntakeReceiveElbowSetpoint = 0;

}
