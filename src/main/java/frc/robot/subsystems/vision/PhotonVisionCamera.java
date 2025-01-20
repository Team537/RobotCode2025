package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Wraps a single PhotonVision camera + a PhotonPoseEstimator. 
 * You will call update() each loop and get the latest robot-pose measurement (if any).
 */
public class PhotonVisionCamera extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    public PhotonVisionCamera(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);

        // Construct a PhotonPoseEstimator
        this.photonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.POSE_STRATEGY,
            robotToCamera // Usually robotToCamera, not cameraToRobot
        );
        
        // Fall back to closest to reference pose strategy when we can only see one tag.
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    /**
     * Should be called once per robot loop. 
     * Provide the current best guess of the robot pose so that the solvePnP can converge better.
     */
    public void update(Pose2d currentBestGuess) {

        // Verify that data is being output.
        System.out.println(this.getName() + " is detecting data!");

        // Update reference pose for better solvePNP
        photonPoseEstimator.setReferencePose(currentBestGuess);

        // Get the latest pipeline result (no need for getAllUnreadResults).
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            // No target, so no pose measurement
            latestEstimatedPose = Optional.empty();
            return;
        }

        // Attempt to get a vision-based global field pose
        var maybePose = photonPoseEstimator.update(result);
        if (maybePose.isPresent()) {
            // Replace its timestamp with the real capture time
            EstimatedRobotPose camEstimatedPose = maybePose.get();
            latestEstimatedPose = Optional.of(camEstimatedPose);
        } else {
            latestEstimatedPose = Optional.empty();
        }
    }

    /**
     * Returns the latest field-relative pose measurement from this camera, if any.
     */
    public Optional<EstimatedRobotPose> getLatestPose() {
        return latestEstimatedPose;
    }

    /**
     * Sets the pipeline that this PhotonVisionCamera will run using.
     * 
     * @param pipelineIndex The pipeline that this PhotonVisionCamera will run using.
     */
    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }
}