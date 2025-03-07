// package frc.robot.subsystems;


// import static edu.wpi.first.units.Units.DegreesPerSecond;

// import java.util.Optional;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.VisionConstants;
// import limelight.Limelight;
// import limelight.networktables.AngularVelocity3d;
// import limelight.networktables.LimelightPoseEstimator;
// import limelight.networktables.Orientation3d;
// import limelight.networktables.PoseEstimate;


// public class VisionSubsystem extends SubsystemBase {
// Limelight limelight;
// LimelightPoseEstimator poseEstimator;
// AHRS navx;
// SwerveSubsystem drivebase;


// Pose3d cameraOffset = new Pose3d(
// VisionConstants.LIMELIGHT_OFFSET_X,
// VisionConstants.LIMELIGHT_OFFSET_Y,
// VisionConstants.LIMELIGHT_OFFSET_Z,
// VisionConstants.LIMELIGHT_OFFSET_ROTATION);

// public VisionSubsystem() {

// navx = new AHRS(NavXComType.kMXP_SPI);

// limelight = new Limelight(VisionConstants.LIMELIGHT_NAME1);
// limelight.getSettings()
// .withCameraOffset(cameraOffset)
// .save();

// poseEstimator = limelight.getPoseEstimator(true);
// }

// // Updates vision estimate (called in swerve subsystem periodic)
// public void updatePoseEstimation() {
// getLimelightSettings();
// getVisionEstimate();
// }

// // Required to be called periodically for MegaTag 2
// public void getLimelightSettings() {
// limelight.getSettings()
// .withRobotOrientation(new Orientation3d(navx.getRotation3d(), new AngularVelocity3d(
// DegreesPerSecond.of(0),
// DegreesPerSecond.of(0),
// DegreesPerSecond.of(0))))
// .save();
// }

// // Adds the vision measurement to the swerve pose estimator
// public void getVisionEstimate() {
// Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
// visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
// // If the average tag distance is less than 4 meters,
// // there are more than 0 tags in view,
// // and the average ambiguity between tags is less than 30% then we update the pose estimation.
// if (poseEstimate.avgTagDist < VisionConstants.AVG_TAG_DIST_FILTER
// && poseEstimate.tagCount > VisionConstants.MIN_TAGS_VISIBLE_FILTER
// && poseEstimate.getMinTagAmbiguity() < 0.3) {
// drivebase.swerveDrivePoseEstimator.addVisionMeasurement(
// poseEstimate.pose.toPose2d(),
// poseEstimate.timestampSeconds);
// }
// });

// }
// }

