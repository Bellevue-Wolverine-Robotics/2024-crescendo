package frc.robot.constants;

import frc.utils.PIDUtils;

public final class FlywheelConstants {
	public static final double kShooterMaxRPM = 5828; // used for calculating FF

	// CAN IDs
	public static final int kArmShoulderId = 8;
	public static final int kArmElbowId = 10;
	public static final int kShooterLeaderId = 11;
	public static final int kShooterFollowerId = 13;
	public static final int kFeederId = 12;

	// PID

	public static final PIDUtils.SparkPIDParams kArmShoulderPid = new PIDUtils.SparkPIDParams(((double) 1 / 19) * 0.3,
			0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kArmElbowPid = new PIDUtils.SparkPIDParams(((double) 1 / 4) * 0.3, 0,
			0, 0, 0, -1, 1);

	// Setpoints

	public static final double kArmShoulderTolerance = 0.5;
	public static final double kArmElbowTolerance = 0.5;

	// Negative duty cycles correspond to shooting
	// Positive duty cycles correspond to feeding in
	public static final double kShootSpeakerDutyCycleSetpoint = -0.5;
	public static final double kFeederDutyCycleSetpoint = 0.2;

	// For the shoulder and elbow, positive encoder and set() values correspond to
	// leaving the
	// robot

	// Negative elbow corresponds to twisting towards the robot, pointing up

	public static final double kSpeakerShoulderSetpoint = 19;
	public static final double kSpeakerElbowSetpoint = -4;

	public static final double kAmpShoulderSetpoint = 0;
	public static final double kAmpElbowSetpoint = 0;

	public static final double kIntakeReceiveShoulderSetpoint = 0;
	public static final double kIntakeReceiveElbowSetpoint = 0;

	public static final int kNoteSwitchDIOPort = 6;

}
