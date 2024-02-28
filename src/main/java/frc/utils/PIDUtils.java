package frc.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class PIDUtils {

	/*public static class CANSparkMaxPID{
		private CANSparkMax thisMotor;
		public CANSparkMaxPID(CANSparkMax motor){
			thisMotor = motor;
		}
		public updateKp(){
			
		}
	}*/


	public static class ArmFFParams {
		public final double kGravity;
		public final double kStatic;
		public final double kVelocity;
		public final double kAcceleration;

		public ArmFFParams(double kGravity, double kStatic, double kVelocity, double kAcceleration) {
			this.kGravity = kGravity;
			this.kStatic = kStatic;
			this.kVelocity = kVelocity;
			this.kAcceleration = kAcceleration;
		}

	}

	public static class WPIPidParams {
		public final double kP;
		public final double kI;
		public final double kD;

		public WPIPidParams(double kP, double kI, double kD) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
		}
	}

	public static class SparkPIDParams {
		public double kP;
		public double kI;
		public double kD;
		public double kIZone;
		public double kFF;
		public double kMinOutput;
		public double kMaxOutput;

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

		public SparkPIDParams(CANSparkMax m_intakeArmPID){
			var pidController = m_intakeArmPID.getPIDController();
			this.kP = pidController.getP();
			this.kI = pidController.getI();
			this.kD = pidController.getD();
			this.kIZone = pidController.getIZone();
			this.kFF = pidController.getFF();
			this.kMinOutput = pidController.getOutputMin();
			this.kMaxOutput = pidController.getOutputMax();	
		}

		public void changeKp(double kP){
			this.kP = kP;
		}
		public void changeKi(double kI){
			this.kI = kI;
		}
		public void changeKd(double kD){
			this.kD = kD;
		}
		public void changeKiZone(double kiZone){
			this.kIZone = kiZone;
		}
		public void changeKff(double kFF){
			this.kFF = kFF;
		}
		public void changekMinOutput(double kMinOutput){
			this.kMinOutput = kMinOutput;
		}
		public void changekMaxOutput(double kMaxOutput){
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

	public static ArmFeedforward createArmFeedforward(ArmFFParams ffParams) {
		return new ArmFeedforward(ffParams.kGravity, ffParams.kStatic, ffParams.kVelocity, ffParams.kAcceleration);
	}

	public static PIDController createPIDController(WPIPidParams pidParams) {
		return new PIDController(pidParams.kP, pidParams.kI, pidParams.kD);
	}


	
}
