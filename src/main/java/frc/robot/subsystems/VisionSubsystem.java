package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
	PhotonCamera camera;
	Transform3d pose3d;
	boolean updated = false;


	public VisionSubsystem() {
		camera = new PhotonCamera("photonvision");
		periodic();
	}

	@Override
	public void periodic() {
		var result = camera.getLatestResult();
		boolean hasTargets = result.hasTargets();

		if(hasTargets){
			PhotonTrackedTarget target = result.getBestTarget();
			pose3d = target.getBestCameraToTarget();
			updated = true;
		}
		else{
			updated = false;
		}

		//List<PhotonTrackedTarget> targets = result.getTargets();
		//PhotonTrackedTarget target = result.getBestTarget();
	}
	
	public Transform3d getTransform3d(){
		return pose3d;
	}

	public boolean hasTarget(){
		return updated;
	}

}
