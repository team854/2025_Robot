package frc.robot.subsystems;

// Import your navX library – adjust this if you’re using a different navX library.
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    // Name to use for Limelight (empty string defaults to "limelight")
    private static final String   LIMELIGHT_NAME = VisionConstants.LIMELIGHT_NAME1;

    // Reference to the swerve subsystem so we can update its pose estimator.
    private final SwerveSubsystem swerveSubsystem;
    // Reference to the navX2 gyro (connected directly to the roboRIO)
    private final AHRS            navx;

    /**
     * Constructs the VisionSubsystem.
     *
     * @param swerveSubsystem Reference to your swerve drive subsystem.
     * @param navx The navX2 gyro instance.
     */
    public VisionSubsystem(SwerveSubsystem swerveSubsystem, AHRS navx) {
        this.swerveSubsystem = swerveSubsystem;
        this.navx            = navx;

        LimelightHelpers.setCameraPose_RobotSpace(
            LIMELIGHT_NAME,
            VisionConstants.LIMELIGHT_OFFSET_FORWARD,
            VisionConstants.LIMELIGHT_OFFSET_LEFT,
            VisionConstants.LIMELIGHT_OFFSET_HEIGHT,
            VisionConstants.LIMELIGHT_ROLL,
            VisionConstants.LIMELIGHT_PITCH,
            VisionConstants.LIMELIGHT_YAW);
    }

    @Override
    public void periodic() {
        updateVisionPose();
    }

    /**
     * Updates the robot’s pose estimate using vision data from the Limelight.
     * This implementation uses MegaTag2: it first sends the current gyro yaw to
     * the Limelight, then retrieves a pose estimate (if valid) and adds it to the
     * swerve drive pose estimator.
     */
    private void updateVisionPose() {
        // Provide the current robot yaw from the navX (in degrees)
        double robotYaw = navx.getYaw();
        // Inform the Limelight of the robot’s orientation (MegaTag2 fusion)
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Retrieve the pose estimate from Limelight (using the default "wpiBlue" coordinate frame)
        LimelightHelpers.PoseEstimate visionMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        // Check if we have a valid vision measurement
        if (visionMeasurement != null && visionMeasurement.pose != null && visionMeasurement.tagCount >= 1) {

            SwerveDrivePoseEstimator poseEstimator = swerveSubsystem.getPoseEstimator();
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));

            // Optionally, adjust the vision timestamp to account for the Limelight’s processing latency.
            double visionLatencySeconds = LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME) / 1000.0;
            double visionTimestamp      = Timer.getFPGATimestamp() - visionLatencySeconds;

            // Update the swerve drive’s pose estimator with the vision measurement.
            poseEstimator.addVisionMeasurement(visionMeasurement.pose, visionTimestamp);
        }
    }
}
