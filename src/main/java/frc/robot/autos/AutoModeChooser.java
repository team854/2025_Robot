package frc.robot.autos;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem          swerveSubsystem;

    public AutoModeChooser(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        autoChooser          = new SendableChooser<>();

        // Default autonomous routine
        autoChooser.setDefaultOption("L4 Auto", swerveSubsystem.getAutonomousCommand("L4 Auto"));

        // Add PathPlanner auto options
        autoChooser.addOption("L4 Auto", swerveSubsystem.getAutonomousCommand("L4 Auto"));
        autoChooser.addOption("Delayed L4 Auto", swerveSubsystem.getAutonomousCommand("Delayed L4 Auto"));
        autoChooser.addOption("Leave Auto", swerveSubsystem.getAutonomousCommand("Leave Auto"));
        autoChooser.addOption("Delayed Leave Auto (5s)", swerveSubsystem.getAutonomousCommand("Delayed Leave Auto (5s)"));
        autoChooser.addOption("Delayed Leave Auto (10s)", swerveSubsystem.getAutonomousCommand("Delayed Leave Auto (10s)"));

        // Add the chooser to Shuffleboard
        Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser)
            .withPosition(0, 0).withSize(2, 1);
    }

    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }
}
