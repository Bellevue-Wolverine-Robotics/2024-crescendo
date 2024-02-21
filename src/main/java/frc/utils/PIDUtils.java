package frc.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.SparkPIDController;

public class PIDUtils {
	public static class SparkPIDParams {
		public final double kP;
		public final double kI;
		public final double kD;
		public final double kIZone;
		public final double kFF;
		public final double kMinOutput;
		public final double kMaxOutput;

		public SparkPIDParams(double kP, double kI, double kD, double kIZone, double kFF, double kMinOutput,
				double kMaxOutput) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.kIZone = kIZone;
			this.kFF = kFF;
			this.kMinOutput = kMinOutput;
			this.kMaxOutput = kMaxOutput;
		}
	}

	public static class TalonPIDParams {
		public final double kP;
		public final double kI;
		public final double kD;
		public final double kIZone;
		public final double kFF;

		public TalonPIDParams(double kP, double kI, double kD, double kIZone, double kFF) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.kIZone = kIZone;
			this.kFF = kFF;
		}
	}

	public static void setPIDConstants(SparkPIDController pidController, SparkPIDParams pidParams) {
		pidController.setP(pidParams.kP);
		pidController.setI(pidParams.kI);
		pidController.setD(pidParams.kD);
		pidController.setIZone(pidParams.kIZone);
		pidController.setFF(pidParams.kFF);
		pidController.setOutputRange(pidParams.kMinOutput, pidParams.kMaxOutput);
	}

	public static void setPIDConstants(SparkPIDController pidController, double kP, double kI, double kD, double kIZone,
			double kFF, double kMinOutput, double kMaxOutput) {
		pidController.setP(kP);
		pidController.setI(kI);
		pidController.setD(kD);
		pidController.setIZone(kIZone);
		pidController.setFF(kFF);
		pidController.setOutputRange(kMinOutput, kMaxOutput);
	}

	public static void setPIDConstants(WPI_TalonSRX talon, TalonPIDParams pidParams) {
		talon.config_kP(0, pidParams.kP);
		talon.config_kI(0, pidParams.kI);
		talon.config_kD(0, pidParams.kD);
		talon.config_IntegralZone(0, pidParams.kIZone);
		talon.config_kF(0, pidParams.kFF);
	}

	public static boolean atSetpoint(double measurement, double setpoint, double tolerance) {
		return Math.abs(measurement - setpoint) < tolerance;
	}
}
