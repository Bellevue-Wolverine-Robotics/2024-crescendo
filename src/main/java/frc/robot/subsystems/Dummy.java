package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dummy extends SubsystemBase {
	public Dummy() {
		BooleanSupplier bsupply = (() -> {
			// Boolean supplier that controls when the path will be mirrored for the red
			// alliance

			return false;
		});



		// Create a path following command using AutoBuilder. This will also trigger event markers.
		AutoBuilder.configureRamsete(
				this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getCurrentSpeeds, // Current ChassisSpeeds supplier
				this::drive, // Method that will drive the robot given ChassisSpeeds
				0.02, // Robot control loop period in seconds. Default is 0.02
				new ReplanningConfig(), 
				bsupply, // Boolean supplier that controls when the path will be mirrored for the red alliance
				this); // Reference to this subsystem to set requirements

	}

	public Pose2d getPose() {
		return new Pose2d();
	}

	public void resetPose(Pose2d pose) {
	}

	public ChassisSpeeds getCurrentSpeeds() {
		return new ChassisSpeeds();
	}

	public void drive(ChassisSpeeds speeds) {
	}

}
