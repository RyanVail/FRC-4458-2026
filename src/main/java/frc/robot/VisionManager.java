package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionManager {
    private static final int NUM_CAMERAS = VisionConstants.CAMERAS.length;
    private static PhotonCamera cameras[] = new PhotonCamera[NUM_CAMERAS];
    private static PhotonPoseEstimator estimators[] = new PhotonPoseEstimator[NUM_CAMERAS];

    public static void initialize() {
        try {
            for (int i = 0; i < NUM_CAMERAS; i++) {
                VisionConstants.Camera camera = VisionConstants.CAMERAS[i];

                cameras[i] = new PhotonCamera(camera.name());
                estimators[i] = new PhotonPoseEstimator(
                        FieldConstants.LAYOUT,
                        camera.transform());
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static List<EstimatedRobotPose> getEstimatedPoses() {
        List<EstimatedRobotPose> poses = new ArrayList<>();

        for (int i = 0; i < NUM_CAMERAS; i++) {
            List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
            for (PhotonPipelineResult r : results) {
                Optional<EstimatedRobotPose> pose = estimators[i].estimateAverageBestTargetsPose(r);
                if (pose.isPresent()) {
                    poses.add(pose.get());
                }
            }
        }

        return poses;
    }
}
