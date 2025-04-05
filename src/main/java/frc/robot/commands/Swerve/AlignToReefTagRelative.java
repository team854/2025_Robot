package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

/**
 * Drives the robot to a fixed pose relative to a "reef" AprilTag using Limelight pose estimation.
 */
public class AlignToReefTagRelative extends Command {
    private final PIDController   xController;
    private final PIDController   yController;
    private final PIDController   rotController;
    private final boolean         isRightScore;
    private final Timer           dontSeeTagTimer = new Timer();
    private final Timer           stopTimer       = new Timer();
    private final SwerveSubsystem drivebase;
    private int                   tagID           = -1;

    public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
        this.isRightScore = isRightScore;
        this.drivebase    = drivebase;

        // PID gains (tunable via AutoAlignConstants.X/Y/ROT_REEF_ALIGNMENT_P)
        xController       = new PIDController(AutoAlignConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
        yController       = new PIDController(AutoAlignConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
        rotController     = new PIDController(AutoAlignConstants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        dontSeeTagTimer.reset();
        dontSeeTagTimer.start();
        stopTimer.reset();
        stopTimer.start();

        // Set desired pose offsets and tolerances from AutoAlignConstants
        rotController.setSetpoint(AutoAlignConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(AutoAlignConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(AutoAlignConstants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(AutoAlignConstants.X_TOLERANCE_REEF_ALIGNMENT);

        double ySet = isRightScore
            ? AutoAlignConstants.Y_SETPOINT_REEF_ALIGNMENT
            : -AutoAlignConstants.Y_SETPOINT_REEF_ALIGNMENT;
        yController.setSetpoint(ySet);
        yController.setTolerance(AutoAlignConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        // Capture the current tag ID so we only track one tag throughout
        tagID = (int) LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME1);
    }

    @Override
    public void execute() {
        String  ll  = VisionConstants.LIMELIGHT_NAME1;
        boolean tv  = LimelightHelpers.getTV(ll);
        int     fid = (int) LimelightHelpers.getFiducialID(ll);

        if (tv && fid == tagID) {
            // Reset loss-of-tag timer
            dontSeeTagTimer.reset();

            // Get the target-space pose array: [x_left, y_up, z_forward, ... , yaw]
            double[] poseArr   = LimelightHelpers.getBotPose_TargetSpace(ll);
            double   xDistance = poseArr[2];
            double   yDistance = poseArr[0];
            double   yawError  = poseArr[4];

            SmartDashboard.putNumber("ReefTag X dist", xDistance);
            SmartDashboard.putNumber("ReefTag Y dist", yDistance);
            SmartDashboard.putNumber("ReefTag Yaw", yawError);

            // Calculate drive outputs
            double xSpeed   = xController.calculate(xDistance);
            double ySpeed   = -yController.calculate(yDistance);
            double rotSpeed = -rotController.calculate(yawError);

            // Command the swerve in field-oriented mode
            drivebase.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

            // If any controller is outside tolerance, reset the on-target dwell timer
            if (!xController.atSetpoint()
                || !yController.atSetpoint()
                || !rotController.atSetpoint()) {
                stopTimer.reset();
            }
        }
        else {
            // Tag lost or wrong ID: stop motion
            drivebase.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
        }

        SmartDashboard.putNumber("PoseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure we stop the robot when the command ends
        drivebase.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // Finish if we've lost the tag too long or held on target long enough
        return dontSeeTagTimer.hasElapsed(AutoAlignConstants.DONT_SEE_TAG_WAIT_TIME)
            || stopTimer.hasElapsed(AutoAlignConstants.POSE_VALIDATION_TIME);
    }
}
