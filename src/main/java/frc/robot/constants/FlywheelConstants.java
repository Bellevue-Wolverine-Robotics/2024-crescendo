package frc.robot.constants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.utils.PIDUtils;
import frc.utils.SparkPIDParams;

public final class FlywheelConstants {
	public static final double kShooterMaxRPM = 5828; // used for calculating FF

	// CAN IDs
	public static final int kShooterLeaderId = 5;
	public static final int kShooterFollowerId = 6;
	public static final int kArmShoulderId = 7;
	public static final int kArmElbowId = 8;

	// PID

	public static final PIDUtils.SparkPIDParams kShooterVelocityPid = new PIDUtils.SparkPIDParams(6e-4, 0, 0, 0,
			1 / kShooterMaxRPM, -1,
			1);

	public static final PIDUtils.SparkPIDParams kArmShoulderPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kArmElbowPid = new PIDUtils.SparkPIDParams(0, 0, 0, 0, 0, -1, 1);

	public static final double kShootStageDutyCycleSetpoint = 0.5;

	public static final double kStageShoulderSetpoint = 0;
	public static final double kStageElbowSetpoint = 0;

	public static final double kAmpShoulderSetpoint = 0;
	public static final double kAmpElbowSetpoint = 00;
}
