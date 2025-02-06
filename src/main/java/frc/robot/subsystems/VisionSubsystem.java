package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.estimator.PoseEstimate;
import limelight.structures.AngularVelocity3d;
import limelight.structures.LimelightSettings.LEDMode;
import limelight.structures.Orientation3d;

public class VisionSubsystem extends SubsystemBase {

    Pose3d                 cameraOffset = new Pose3d(Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Rotation3d.kZero);

    Limelight              limelight;
    LimelightPoseEstimator poseEstimator;

    AHRS                   navx         = new AHRS(NavXComType.kMXP_SPI);


    public VisionSubsystem() {
        limelight = new Limelight("limelight");
        limelight.settingsBuilder()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(cameraOffset)
            .save();
        poseEstimator = limelight.getPoseEstimator(true);

    }

    @Override
    public void periodic() {

        // Required for megatag2
        limelight.settingsBuilder()
            .withRobotOrientation(new Orientation3d(navx.getRotation3d(),
                new AngularVelocity3d(DegreesPerSecond.of(0),
                    DegreesPerSecond.of(0),
                    DegreesPerSecond.of(0))))
            .save();

        // Get the vision estimate.
        Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
        visionEstimate.ifPresent(poseEstimate -> {
            // If the average tag distance is less than 4 meters,
            // there are more than 0 tags in view,
            // and the average ambiguity between tags is less than 30%, then we update the pose estimation.
            if (poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.3) {



            }
        });

        // limelight.getLatestResults().ifPresent((LimelightResults result) -> {

        // // example object detection
        // for (NeuralClassifier object : result.targets_Classifier) {
        // if (object.className.equals("algae")) {
        // if (object.ty > 2 && object.ty < 1) {
        // // do stuff
        // }
        // }
        // }
        // });
    }
}