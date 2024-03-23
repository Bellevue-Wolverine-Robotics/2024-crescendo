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
	// public static final double kThrottlePreset1 = 0.5;
	// public static final double kRotationPreset1 = 0.3;

	// public static final double kThrottlePreset2 = 0.2;
	// public static final double kRotationPreset2 = 0.2;

	public static final int kThrottle1Button = 3;
	public static final double kThrottle1Speed = 2;

	public static final int kThrottle2Button = 5;
	public static final double kThrottle2Speed = 3;

	public static final int kThrottle3Button = 2;
	public static final double kThrottle3Speed = 0.5;




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
	public static final double ff = (0.7578125 / 2.930143117904663);

	public static final SparkPIDParams kDriveVelocityPIDParams = new SparkPIDParams(0.5, 0, 0, 0, 1 / 6.5,
			-1, 1);

	public static final double kMaxSimInputVoltage = 12;
	public static final double rotationIntensity = 0.5;
    public static double aboveSpeedThreshold = 0.05;

}
