package frc.robot.autos;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/*
    Jonathan Graydon's custom auto mode chooser!
    This program will allow you to easily set the desired auto route even while the robot is on the field
    Will take a second to show up usually as the robot boots up on the field
    Requires path planner to be set up correctly in the swerve drive subsystem

    Steps for setting up:
    1. Set default auto which will run if nothing else has been selected (put something safe here)
    2. Create autos in path planner
    3. Use autoChooser.addOption to add the pathplanner auto to the list of available autos on the driver dashboard
    NOTE: The "name:" param can be set to anything but the "pathName:" param must exactly match the auto path on pathplanner
 */

public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem          swerveSubsystem;

    public AutoModeChooser(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        autoChooser          = new SendableChooser<>();

        // Default autonomous routine
        autoChooser.setDefaultOption("Leave Auto", swerveSubsystem.getAutonomousCommand("Leave Auto"));

        // Add PathPlanner auto options
        autoChooser.addOption("L4 Auto", swerveSubsystem.getAutonomousCommand("L4 Auto"));
        autoChooser.addOption("Delayed L4 Auto", swerveSubsystem.getAutonomousCommand("Delayed L4 Auto"));
        autoChooser.addOption("Leave Auto", swerveSubsystem.getAutonomousCommand("Leave Auto"));
        autoChooser.addOption("Delayed Leave Auto (5s)", swerveSubsystem.getAutonomousCommand("Delayed Leave Auto (5s)"));
        autoChooser.addOption("Delayed Leave Auto (10s)", swerveSubsystem.getAutonomousCommand("Delayed Leave Auto (10s)"));
        autoChooser.addOption("Side Auto", swerveSubsystem.getAutonomousCommand("TEST Side Reef Auto"));
        autoChooser.addOption("TEST Straight Path Auto", swerveSubsystem.getAutonomousCommand("TEST Straight Path Auto"));

        // Add the chooser to Shuffleboard
        Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser)
            .withPosition(0, 0).withSize(2, 1);
    }

    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }
}
