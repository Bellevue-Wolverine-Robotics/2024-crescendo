package frc.robot.constants;

import frc.utils.PIDUtils.SparkPIDParams;

public final class ClimberConstants {
	public static final int kLeftClimbMotorId = 5;
	public static final int kRightClimbMotorId = 6;

	public static final int kLimitSwitchDIOPort = 1;

	public static final SparkPIDParams kClimbPidParams = new SparkPIDParams(1, 0, 0, 0, 0, -1, 1);

	public static final double kClimbExtendedSetpoint = 40.0;
	public static final double kClimbRetractedSetpoint = 0.0;
	public static final double kClimbTolerance = (0.05) * kClimbRetractedSetpoint;

	public static final double kPositionConversionFactor = 1.0; // depends on gear ratio and diameter

	public static final int kSmartCurrentLimit = 30;
}