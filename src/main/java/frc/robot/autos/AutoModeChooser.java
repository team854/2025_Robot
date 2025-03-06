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
        autoChooser.setDefaultOption("1x Trough", swerveSubsystem.getAutonomousCommand("1x Trough Auto"));

        // Add PathPlanner auto options
        autoChooser.addOption("1x Trough Auto", swerveSubsystem.getAutonomousCommand("1x Trough Auto"));
        autoChooser.addOption("4x L4 Auto", swerveSubsystem.getAutonomousCommand("4x L4 Coral Auto"));
        autoChooser.addOption("2x L4 Auto", swerveSubsystem.getAutonomousCommand("2x L4 Auto"));

        // Add the chooser to Shuffleboard
        Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser)
            .withPosition(0, 0).withSize(2, 1);
    }

    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }
}
