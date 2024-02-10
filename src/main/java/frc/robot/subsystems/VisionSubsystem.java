package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
	AprilTagDetector m_aprilTagDetector;
	AprilTagPoseEstimator m_aprilTagPoseEstimator;
	CvSink m_cvSink;
	CvSource m_outputStream;
	Mat m_mat;
	Mat m_grayMat;

	ArrayList<Long> m_tags;

	Scalar m_outlineColor;
	Scalar m_crossColor;

	NetworkTable m_tagsTable;
	IntegerArrayPublisher m_pubTags;

	public VisionSubsystem() {
		m_aprilTagDetector = new AprilTagDetector();
		m_aprilTagDetector.addFamily("tag36h11", 3); // TODO: idk what this means

		// Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
		// (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
		Config poseEstConfig = new AprilTagPoseEstimator.Config(
				0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
		m_aprilTagPoseEstimator = new AprilTagPoseEstimator(poseEstConfig);

		UsbCamera camera = CameraServer.startAutomaticCapture();
		camera.setResolution(640, 480);

		m_cvSink = CameraServer.getVideo();
		m_outputStream = CameraServer.putVideo("Detected", 640, 480);

		// Mats are very memory expensive. Lets reuse these.
		m_mat = new Mat();
		m_grayMat = new Mat();

		// Instantiate once
		m_tags = new ArrayList<>();
		m_outlineColor = new Scalar(0, 255, 0);
		m_crossColor = new Scalar(0, 0, 255);

		// We'll output to NT
		m_tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
		m_pubTags = m_tagsTable.getIntegerArrayTopic("tags").publish();
	}

	@Override
	public void periodic() {
		// Tell the CvSink to grab a frame from the camera and put it
		// in the source mat. If there is an error notify the output.
		if (m_cvSink.grabFrame(m_mat) == 0) {
			// Send the output the error.
			m_outputStream.notifyError(m_cvSink.getError());
			// skip the rest of the current iteration
			return;
		}

		Imgproc.cvtColor(m_mat, m_grayMat, Imgproc.COLOR_RGB2GRAY);

		AprilTagDetection[] detections = m_aprilTagDetector.detect(m_grayMat);

		// have not seen any tags yet
		m_tags.clear();

		for (AprilTagDetection detection : detections) {
			// remember we saw this tag
			m_tags.add((long) detection.getId());

			// draw lines around the tag
			for (var i = 0; i <= 3; i++) {
				var j = (i + 1) % 4;
				var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
				var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
				Imgproc.line(m_mat, pt1, pt2, m_outlineColor, 2);
			}

			// mark the center of the tag
			var cx = detection.getCenterX();
			var cy = detection.getCenterY();
			var ll = 10;
			Imgproc.line(m_mat, new Point(cx - ll, cy), new Point(cx + ll, cy), m_crossColor, 2);
			Imgproc.line(m_mat, new Point(cx, cy - ll), new Point(cx, cy + ll), m_crossColor, 2);

			// identify the tag
			Imgproc.putText(
					m_mat,
					Integer.toString(detection.getId()),
					new Point(cx + ll, cy),
					Imgproc.FONT_HERSHEY_SIMPLEX,
					1,
					m_crossColor,
					3);

			// determine pose
			Transform3d pose = m_aprilTagPoseEstimator.estimate(detection);

			// put pose into dashboard
			Rotation3d rot = pose.getRotation();
			m_tagsTable
					.getEntry("pose_" + detection.getId())
					.setDoubleArray(
							new double[] {
									pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
							});
		}

		// put list of tags onto dashboard
		m_pubTags.set(m_tags.stream().mapToLong(Long::longValue).toArray());

		// Give the output stream a new image to display
		m_outputStream.putFrame(m_mat);
	}

}
