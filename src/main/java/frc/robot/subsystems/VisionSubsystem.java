package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
	PhotonCamera camera;
	Transform3d pose3d;
	boolean updated = false;
	AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	PhotonPoseEstimator photonPoseEstimator;

	double updatedTime;
	Pose2d robotPos;

	public VisionSubsystem() {
		camera = new PhotonCamera("photonvision");
		periodic();
	}

	@Override
	public void periodic() {
		// var result = camera.getLatestResult();
		// boolean hasTargets = result.hasTargets();

		// if(hasTargets){
		// pose3d = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new
		// Rotation3d(0,0,0));

		// photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
		// PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, pose3d);

		// updated = true;
		// }
		// else{
		// updated = false;
		// }
		// //List<PhotonTrackedTarget> targets = result.getTargets();
		// //PhotonTrackedTarget target = result.getBestTarget();
		// }

		// public Transform3d getTransform3d(){
		// return pose3d;
		// }

		// public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d
		// prevEstimatedRobotPose) {
		// if(updated){
		// photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		// Optional<EstimatedRobotPose> res = photonPoseEstimator.update();

		// robotPos = res.get().estimatedPose.toPose2d();
		// updatedTime = res.get().timestampSeconds;

		// return res;
		// }
		// return null;
	}

	public Pose2d getPose2d() {
		return robotPos;
	}

	public double getTimestampSeconds() {
		return updatedTime;
	}

	public boolean hasTarget() {
		return updated;
	}

}
