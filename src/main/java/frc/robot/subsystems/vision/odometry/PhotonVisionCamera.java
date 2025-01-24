package frc.robot.subsystems.vision.odometry;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Wraps a single PhotonVision camera + a PhotonPoseEstimator. 
 * You will call update() each loop and get the latest robot-pose measurement (if any).
 * <hr>
 * @author Cameron Myhre
 * @since v1.2.0
 */
public class PhotonVisionCamera extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;

    // Stores our most recently computed estimated pose (if any).
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    /**
     * Create a new PhotonVisionCamera object using the camera name and the RobotToCamera Transformation.
     * 
     * @param cameraName The name of the camera being used. This value is the same as what the name was set to on the Orange PI.
     * @param robotToCamera The Transform3d object representing the cameras offset from the robot's origin.
     */
    public PhotonVisionCamera(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);

        // Construct a PhotonPoseEstimator
        this.photonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.POSE_STRATEGY,
            robotToCamera // Usually robotToCamera, not cameraToRobot
        );
        
        // Fall back to a second strategy if only one tag is visible
        photonPoseEstimator.setMultiTagFallbackStrategy(VisionConstants.FALLBACK_STRATEGY);
    }

    /**
     * Should be called once per robot loop.
     * Provide the current best guess of the robot pose so that solvePnP can converge better.
     */
    public void update(Pose2d currentBestGuess) {

        // Update the reference pose for better solvePNP
        photonPoseEstimator.setReferencePose(currentBestGuess);

        // Grab *all* the new pipeline results since our last call
        List<PhotonPipelineResult> allResults = camera.getAllUnreadResults();

        if (allResults.isEmpty()) {
            // No new frames arrived this loop
            latestEstimatedPose = Optional.empty();
            return;
        }

        // Sort them in ascending order by capture time so we process chronologically
        allResults.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));

        // We'll track the last valid pose we get from any of these frames
        Optional<EstimatedRobotPose> newestPose = Optional.empty();

        // Process each frame in chronological order
        for (PhotonPipelineResult result : allResults) {
            if (!result.hasTargets()) {
                // Skip frames with no targets
                continue;
            }
            // Attempt to get a vision-based global field pose
            var maybePose = photonPoseEstimator.update(result);
            if (maybePose.isPresent()) {
                // If we got a valid pose, store it.
                newestPose = maybePose;
            }
        }

        latestEstimatedPose = newestPose;
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
     * @param pipelineIndex The pipeline index to switch to.
     */
    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }
}