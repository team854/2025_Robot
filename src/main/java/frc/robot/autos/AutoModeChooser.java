package frc.robot.autos;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoModeChooser {

    private final SendableChooser<Command> autoChooser;

    public AutoModeChooser() {
        autoChooser = new SendableChooser<>();

        // Add autonomous options
        // autoChooser.setDefaultOption("Default Auto", new DefaultAutoCommand());

        // Add the chooser to Shuffleboard
        Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser)
            .withPosition(0, 0).withSize(2, 1);
    }

    // Method to return the selected autonomous command
    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }
}
