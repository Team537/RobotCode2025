package frc.robot.subsystems.vision.odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h2> VisionOdometry </h2>
 * VisionOdometry is responsible for integrating vision-based robot pose
 * estimates with drivetrain odometry using WPILib's SwerveDrivePoseEstimator.
 * <p>
 * This subsystem gathers pose data from multiple PhotonVision cameras,
 * processes the data to determine the robot's position on the field, and feeds the
 * results into the pose estimator to improve accuracy.
 * <hr>
 * @author Cameron Myhre
 * @since v1.2.0
 */
public class VisionOdometry extends SubsystemBase {

    // The pose estimator used to fuse vision measurements with drivetrain odometry
    private SwerveDrivePoseEstimator poseEstimator;

    // List of PhotonVisionCamera objects, representing cameras mounted on the robot
    private List<PhotonVisionCamera> cameras = new ArrayList<>();
    private Field2d field2d;

    /**
     * Constructs a VisionOdometry subsystem.
     * 
     * @param poseEstimator the shared SwerveDrivePoseEstimator instance responsible
     *                      for maintaining and updating the robot's pose.
     */
    public VisionOdometry(SwerveDrivePoseEstimator poseEstimator) {
        
        // Store the pose estimator object.
        this.poseEstimator = poseEstimator;

        // Display the field on the screen for debugging purposes.
        this.field2d = new Field2d();
    
        SmartDashboard.putData("Field", field2d);
    }

    /**
     * Adds a new PhotonVisionCamera to this VisionOdometry subsystem.
     * <p>
     * Each camera added to this subsystem will contribute to the vision-based
     * pose estimation by providing its measurements during periodic updates.
     * 
     * @param camera the PhotonVisionCamera to add.
     */
    public void addCamera(PhotonVisionCamera camera) {
        cameras.add(camera);
    }

     /**
     * Periodically updates the robot's pose estimator with vision measurements 
     * from all registered cameras.
     * <p>
     * This method is called automatically by the scheduler. For each camera:
     * <ul>
     *   <li>The current best estimate of the robot's pose is passed to the camera.</li>
     *   <li>If the camera provides a valid vision-based pose estimate, it is 
     *       added to the pose estimator with an appropriate timestamp.</li>
     * </ul>
     */
    public void updateVisionPositionData() {

        // Get the current best estimate of the robot's pose from the pose estimator
        Pose2d currentBestPose = poseEstimator.getEstimatedPosition();

        // Iterate through all registered cameras
        for (PhotonVisionCamera camera : cameras) {

            // Update the camera with the current best pose as a reference
            camera.update(currentBestPose);

            // Attempt to get the latest vision-based pose estimate from the camera
            Optional<EstimatedRobotPose> visionPose = camera.getLatestPose();
            if (visionPose.isPresent()) {
                
                // If a valid pose is available, retrieve it
                EstimatedRobotPose estimatedPose = visionPose.get();

                // Add the vision measurement to the pose estimator
                // This fuses the vision data into the overall robot pose
                poseEstimator.addVisionMeasurement(
                        estimatedPose.estimatedPose.toPose2d(), // Convert Pose3d to Pose2d
                        estimatedPose.timestampSeconds // Use the correct timestamp
                );
            }
        }
    }

    /**
     * Periodically updates the robot's pose estimator with vision measurements 
     * from all registered cameras.
     * <p>
     * This method is called automatically by the scheduler. For each camera:
     * <ul>
     *   <li>The current best estimate of the robot's pose is passed to the camera.</li>
     *   <li>If the camera provides a valid vision-based pose estimate, it is 
     *       added to the pose estimator with an appropriate timestamp.</li>
     * </ul>
     */
    @Override
    public void periodic() {

        // Display the robot's current position on the field.
        this.field2d.setRobotPose(this.poseEstimator.getEstimatedPosition());
    }
}
