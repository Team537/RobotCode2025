package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionCamera extends SubsystemBase {

    private PhotonCamera camera;
    private Transform3d cameraOffset; // The camera's position relative to the robot.
    private PhotonPipelineResult result;
    private PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonPipelineResult> currentPhotonPipelineResults;
    private Optional<EstimatedRobotPose> estimatedRobotPoseOptional;

    /**
     * Creates a {@code PhotonVisionCamera} with the desired parameters.
     * 
     * @param cameraName   The name of the camera you want to create.
     * @param cameraOffset The camera's position relative to the robot's origin.
     */
    public PhotonVisionCamera(String cameraName, Transform3d cameraOffset) {

        // Initialize the PhotonCamera object with the given name.
        this.camera = new PhotonCamera(cameraName);

        // Set the camera offset.
        this.cameraOffset = cameraOffset;

        // Initialize the PhotonPoseEstimator for estimating the robot's position.
        this.photonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            this.cameraOffset
        );
    }

    /**
     * Take a snapshot of the unprocessed camera feed.
     * Useful for gathering training data for AI or interesting mid-match photographs.
     */
    public void takeInputSnapshot() {
        this.camera.takeInputSnapshot();
    }

    /**
     * Take a snapshot of the processed camera feed.
     * Useful for gathering interesting mid-match photographs.
     */
    public void takeOutputSnapshot() {
        this.camera.takeOutputSnapshot();
    }

    /**
     * Estimates the robot's position on the field using all visible AprilTags.
     * 
     * @return An estimate of the robot's position on the field.
     */
    public Optional<EstimatedRobotPose> estimateRobotPose() {
        return this.estimatedRobotPoseOptional;
    }

    /**
     * Sets the pipeline that this camera's stream will be processed using.
     * 
     * @param pipeline The pipeline number (0-9) to use for processing the image.
     */
    public void setPipeline(int pipeline) {
        this.camera.setPipelineIndex(pipeline);
    }

    /**
     * Returns a PhotonTrackedTarget with the desired ID if visible, otherwise null.
     * 
     * @param id The ID of the tag to check for visibility.
     * @return The PhotonTrackedTarget if visible, otherwise null.
     */
    public PhotonTrackedTarget getDistanceToTag(int id) {
        
        // Ensure the camera is on the AprilTag detection pipeline.
        if (this.camera.getPipelineIndex() != VisionConstants.APRIL_TAG_PIPELINE) {
            System.err.println("Error: " + this.camera.getName() + " attempted to detect AprilTag using non-AprilTag pipeline. Current pipeline number is " + this.camera.getPipelineIndex() + ".");
            return null;
        }

        // Check if there are any targets detected.
        if (!this.hasTargets()) {
            return null;
        }

        // Check if any of the visible targets have the desired ID.
        List<PhotonTrackedTarget> targets = getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }

        // No target with the desired ID is visible.
        return null;
    }

    /**
     * Returns a list of all targets detected by this camera.
     * 
     * @return A list of PhotonTrackedTargets currently detected.
     */
    public List<PhotonTrackedTarget> getTargets() {
        if (!hasTargets()) {
            return null;
        }
        return result.getTargets();
    }

    /**
     * Returns the most prominent target visible by this camera.
     * 
     * @return The best target visible, or null if no targets are visible.
     */
    public PhotonTrackedTarget getPrimaryTarget() {
        if (!hasTargets()) {
            return null;
        }
        return result.getBestTarget();
    }

    /**
     * Checks if the camera is currently viewing the desired AprilTag.
     * 
     * @param id The ID of the AprilTag to check for visibility.
     * @return True if the AprilTag with the desired ID is visible, otherwise false.
     */
    public boolean hasTargetWithId(int id) {
        if (!hasTargets()) {
            return false;
        }

        List<PhotonTrackedTarget> targets = getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return true;
            }
        }

        return false;
    }

    /**
     * Checks if the camera is detecting any targets.
     * 
     * @return True if a target is visible, otherwise false.
     */
    public boolean hasTargets() {
        return this.result.hasTargets();
    }

    /**
     * Returns the latency of the pipeline in milliseconds.
     * 
     * @return The latency of the pipeline in milliseconds.
     */
    public double getLatencyMs() {
        return this.result.metadata.getLatencyMillis();
    }

    /**
     * Returns the name of this {@code PhotonVisionCamera}.
     * 
     * @return The name of this camera.
     */
    public String getCameraName() {
        return this.camera.getName();
    }

    /**
     * Periodically update the camera's result and estimated robot position.
     */
    @Override
    public void periodic() {

        // Update the camera's result with the most recent data.
        this.currentPhotonPipelineResults = this.camera.getAllUnreadResults();

        // Get the most recent detection.
        if (!this.currentPhotonPipelineResults.isEmpty()) {
            this.result = this.currentPhotonPipelineResults.get(0);

            // Update the estimated robot position.
            this.estimatedRobotPoseOptional = this.photonPoseEstimator.update(this.result);
        }

        // Output estimated position for the purposes of debugging.
        if (this.estimatedRobotPoseOptional.isEmpty()) {
            return;
        }

        // Display the estimated position for debugging purposes.
        EstimatedRobotPose robotPose = estimatedRobotPoseOptional.get();
        SmartDashboard.putNumber("Vision Estimated X: ", robotPose.estimatedPose.getX());
        SmartDashboard.putNumber("Vision Estimated Y: ", robotPose.estimatedPose.getY());
        SmartDashboard.putNumber("Vision Estimated Z: ", robotPose.estimatedPose.getZ());
    }
}
