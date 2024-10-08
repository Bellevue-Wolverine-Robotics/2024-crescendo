package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.utils.PIDUtils.SparkPIDParams;

public final class ClimberConstants {
	public static final int kLeftClimbMotorId = 5;
	public static final int kRightClimbMotorId = 6;

	public static final int kBottomLimitSwitchDIOPort = 8;
	public static final int kTopLimitSwitchDIOPort = 9;

	// public static final SparkPIDParams kClimbPidParams = new SparkPIDParams(2, 0,
	// 0.2, 0, 0, -1, 1);
	public static final SparkPIDParams kClimbPidParams = new SparkPIDParams(1.455, 0.000, 0, 0, 0, -1, 1);

	// units are in meters
	public static final double kClimbExtendedSetpoint = -0.001;
	public static final double kClimbRetractedSetpoint = -0.331;
	public static final double kClimbTolerance = Math.abs((0.01) * kClimbRetractedSetpoint);

	public static final int kSmartCurrentLimit = 30;

	public static final double kSprocketDiameterInches = 3.815;
	public static final double kSprocketDiameterMeters = Units.inchesToMeters(kSprocketDiameterInches);
	public static final double kSprocketCircumferenceMeters = Math.PI * kSprocketDiameterMeters;
	public static final double kGearRatio = ((double) 1 / 30);

	public static final double kClimberPositionConversion = kGearRatio * kSprocketCircumferenceMeters;

	public static final double kControllerClimberSensitivity = (double) 1 / 7;
	public static final double kClimberResetSpeed = 0.3;
}