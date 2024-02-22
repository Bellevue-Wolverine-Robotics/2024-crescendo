package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.utils.PIDUtils.SparkPIDParams;
import frc.utils.PIDUtils.WPIPidParams;

public final class DriveConstants {
	// CAN IDs
	public static final int kFrontLeftMotorId = 4;
	public static final int kFrontRightMotorId = 2;
	public static final int kBackLeftMotorId = 3;
	public static final int kBackRightMotorId = 1;

	// Speed Presets
	public static final double kThrottlePreset1 = 0.5;
	public static final double kRotationPreset1 = 0.3;

	public static final double kThrottlePreset2 = 0.2;
	public static final double kRotationPreset2 = 0.2;

	public static final double kDriveGearRatio = 8.45;
	public static final double kWheelDiameterInches = 6;
	public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
	public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelCircumferenceInches);

	public static final WPIPidParams kTurnPidParams = new WPIPidParams(0.01, 0, 0);
	public static final WPIPidParams kStraightPidParams = new WPIPidParams(2, 0, 0);

	public static final double kTrackWidthInches = 19.75;
	public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

	// TODO: modify this value when we change the wheels and stuff because THIS
	// NUMBER WILL CHANGE
	public static final double kMaxVelocity = 5.4;

	public static final SparkPIDParams kDriveVelocityPIDParams = new SparkPIDParams(0.0009, 0, 0, 0, 1 / kMaxVelocity,
			-1, 1);

	public static final double kMaxSimInputVoltage = 12;

}
