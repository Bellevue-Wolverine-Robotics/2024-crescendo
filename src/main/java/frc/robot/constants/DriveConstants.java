package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.utils.PIDUtils.SparkPIDParams;
import frc.utils.PIDUtils.WPIPidParams;

public final class DriveConstants {
	// CAN IDs
	public static final int frontLeftId = 1;
	public static final int frontRightId = 3;
	public static final int backLeftId = 2;
	public static final int backRightId = 4;

	// Speed Presets
	public static final double kThrottlePreset1 = 0.5;
	public static final double kRotationPreset1 = 0.3;

	public static final double kThrottlePreset2 = 0.2;
	public static final double kRotationPreset2 = 0.2;

	public static final double DRIVE_GEAR_RATIO = 8.45;
	public static final double WHEEEL_CIRCUMFERENCE_INCHES = 18.875;
	public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEEL_CIRCUMFERENCE_INCHES);

	public static final WPIPidParams kTurnPidParams = new WPIPidParams(0.01, 0, 0);
	public static final WPIPidParams kStraightPidParams = new WPIPidParams(2, 0, 0);

	public static final double kTrackWidthInches = 22;
	public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

	// TODO: modify this value when we change the wheels and stuff because THIS
	// NUMBER WILL CHANGE
	public static final double kMaxVelocity = 5.4;

	public static final SparkPIDParams kDriveVelocityPIDParams = new SparkPIDParams(0.0009, 0, 0, 0, 1 / kMaxVelocity,
			-1, 1);

	public static final double kMaxSimInputVoltage = 12;

}
