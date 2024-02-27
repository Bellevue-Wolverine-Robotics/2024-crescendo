package frc.robot.constants;

import frc.utils.PIDUtils.ArmFFParams;
import frc.utils.PIDUtils.SparkPIDParams;

public final class IntakeConstants {
	public static final int kFeederMotorId = 8;
	public static final int kArmMotorId = 7;

	public static final int kSmartCurrentLimit = 30;

	public static final double kArmPositionConversionFactor = 1.0; // depends on gear ratio and diameter

	public static final double kFeederDutyCycle = 0.3;

	public static final SparkPIDParams kIntakeArmPIDParams = new SparkPIDParams(1.0, 0, 0, 0, 0, 1.0, -1.0);

	public static final double kIntakeArmSetpointStow = 0.0;
	public static final double kIntakeArmSetpointDeploy = 1.0;

	public static final ArmFFParams kIntakeArmFFParams = new ArmFFParams(0, 0, 0, 0);

	public static final int kNoteSwitchDIOPort = 0;
	public static final int kFlyWheelwitchDIOPort = 1;
}
