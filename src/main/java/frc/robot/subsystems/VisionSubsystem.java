package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private static final String            LIMELIGHT_NAME = VisionConstants.LIMELIGHT_NAME1;

    private final SwerveSubsystem          swerveSubsystem;
    private final AHRS                     navx;
    private final SwerveDrivePoseEstimator poseEstimator;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem, AHRS navx) {
        this.swerveSubsystem = swerveSubsystem;
        this.navx            = navx;

        // Tell Limelight where it sits on the robot
        LimelightHelpers.setCameraPose_RobotSpace(
            LIMELIGHT_NAME,
            VisionConstants.LIMELIGHT_OFFSET_FORWARD,
            VisionConstants.LIMELIGHT_OFFSET_LEFT,
            VisionConstants.LIMELIGHT_OFFSET_HEIGHT,
            VisionConstants.LIMELIGHT_ROLL,
            VisionConstants.LIMELIGHT_PITCH,
            VisionConstants.LIMELIGHT_YAW);

        // Grab the estimator and set vision noise once
        this.poseEstimator = swerveSubsystem.getPoseEstimator();
        this.poseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5.0)));
    }

    @Override
    public void periodic() {
        updateVisionPose();

        SmartDashboard.putNumber("Vision/Tag Count", vision.tagCount);
        SmartDashboard.putNumber("Vision/Latency", latency);
        SmartDashboard.putNumber("Vision/X", vision.pose.getX());
        SmartDashboard.putNumber("Vision/Y", vision.pose.getY());
        SmartDashboard.putNumber("Vision/Theta", vision.pose.getRotation().getDegrees());

    }

    private void updateVisionPose() {
        // Update Limelight with current yaw
        double robotYaw = navx.getYaw();
        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            robotYaw,
            0, 0, 0, 0, 0);

        // Ask Limelight for a pose
        LimelightHelpers.PoseEstimate vision = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
        if (vision != null && vision.pose != null && vision.tagCount >= 1) {
            // Account for pipeline latency
            double latency   = LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME) / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latency;

            // Fuse into estimator
            poseEstimator.addVisionMeasurement(vision.pose, timestamp);
        }
    }
}
