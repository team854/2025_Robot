package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.OperatorInput;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;


public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoPattern autoPattern = operatorInput.getAutoPattern();
        double      autoDelay   = operatorInput.getAutoDelay();

        Alliance    alliance    = DriverStation.getAlliance().orElse(null);

        if (alliance == null) {
            System.out.println("*** ERROR **** unknown Alliance ");
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Alliance      : ").append(alliance);
        sb.append("\n   Auto Pattern  : ").append(autoPattern);
        sb.append("\n   Delay         : ").append(autoDelay);

        System.out.println(sb.toString());

        // If any inputs are null, then there was some kind of error.
        if (autoPattern == null) {
            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        /*
         * Delay
         */
        if (autoDelay != 0) {
            addCommands(new WaitCommand(autoDelay));
        }

        /*
         * Compose the appropriate auto commands
         */
        switch (autoPattern) {

        case DO_NOTHING:
            return;

        case DRIVE_FORWARD:

            // Set the current heading to zero, the gyro could have drifted while
            // waiting for auto to start.
            driveSubsystem.setGyroHeading(0);

            // Drive forward 1m at .2 speed
            addCommands(new DriveOnHeadingCommand(0, .2, 100, driveSubsystem));
            return;

        case BOX:

            // Set the current heading to zero, the gyro could have drifted while
            // waiting for auto to start.
            driveSubsystem.setGyroHeading(0);

            // Drive out and then one box
            addCommands(new DriveOnHeadingCommand(0, .4, 200, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(270, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(180, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(90, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(0, .4, 100, driveSubsystem));
            return;
        }
    }
}
