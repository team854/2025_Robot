package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.Elastic;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final File                     directory       = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive              swerveDrive;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final AHRS                     navx            = new AHRS(NavXComType.kMXP_SPI);
    private final Rotation3d               gyroOffset      = new Rotation3d(
        0.0,
        0.0,
        Units.degreesToRadians(OperatorConstants.GYRO_OFFSET));

    private final Elastic.Notification     nullAutoWarning = new Elastic.Notification(
        Elastic.Notification.NotificationLevel.WARNING,
        "No Auto Selected",
        "No auto is currently selected, auto will not run");

    public SwerveSubsystem() {
        // High‑verbosity telemetry
        swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Build the Swervelib drive from JSON
        try {
            swerveDrive = new SwerveParser(directory)
                .createSwerveDrive(
                    OperatorConstants.MAX_SPEED,
                    new Pose2d(
                        new Translation2d(Meter.of(1), Meter.of(4)),
                        Rotation2d.fromDegrees(0)));
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Configure gyro offset and velocity compensation
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setGyroOffset(gyroOffset);

        // Initialize WPILib pose estimator (use Swervelib's internal kinematics field)
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            swerveDrive.kinematics,
            navx.getRotation2d(),
            swerveDrive.getModulePositions(),
            swerveDrive.getPose(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2.0)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5.0)));

        // PathPlanner setup
        setupPathPlanner();
    }

    @Override
    public void periodic() {
        // 1) Fuse wheel encoders + gyro into estimator
        swerveDrivePoseEstimator.update(
            navx.getRotation2d(),
            swerveDrive.getModulePositions());

        // 2) Push fused pose back into Swervelib for all consumers
        Pose2d fused = swerveDrivePoseEstimator.getEstimatedPosition();
        swerveDrive.resetOdometry(fused);
    }

    /** Zero the gyro heading to 0. */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /** Slow down maximum speeds (e.g. for precision modes). */
    public void slowSpeed(double linear, double angular) {
        swerveDrive.setMaximumAttainableSpeeds(linear, angular);
    }

    /** @return the WPILib pose estimator (for vision fusion). */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swerveDrivePoseEstimator;
    }

    /** @return the underlying Swervelib drive (for manual control). */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /** Field‑oriented drive (robot‑relative speeds). */
    public void driveFieldOriented(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }

    /** Field‑oriented drive command (for Command‑based). */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
        return run(() -> swerveDrive.driveFieldOriented(speeds.get()));
    }

    private void setupPathPlanner() {
        try {
            RobotConfig config   = RobotConfig.fromGUISettings();
            boolean     enableFF = true;

            AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (robotSpeeds, moduleFF) -> {
                    if (enableFF) {
                        swerveDrive.drive(
                            robotSpeeds,
                            swerveDrive.kinematics.toSwerveModuleStates(robotSpeeds),
                            moduleFF.linearForces());
                    }
                    else {
                        swerveDrive.setChassisSpeeds(robotSpeeds);
                    }
                },
                new PPHolonomicDriveController(
                    new PIDConstants(5, 0, 0),
                    new PIDConstants(5, 0, 0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    /** Create a PathPlannerAuto for the given path. */
    public Command getAutonomousCommand(String pathName) {
        if (pathName == null) {
            Elastic.sendNotification(nullAutoWarning);
        }
        return new PathPlannerAuto(pathName);
    }
}
