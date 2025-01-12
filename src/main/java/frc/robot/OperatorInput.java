package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.GameController;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The DriverController exposes all driver functions
 * <p>
 * Extend SubsystemBase in order to have a built in periodic call to support SmartDashboard updates
 */
public class OperatorInput extends SubsystemBase {

    private final GameController driverController;

    // Auto Setup Choosers
    SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();
    SendableChooser<Integer>     waitTimeChooser    = new SendableChooser<>();
    SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();

    /**
     * Construct an OperatorInput class that is fed by a DriverController and optionally an
     * OperatorController.
     */
    public OperatorInput() {

        driverController = new GameController(OperatorInputConstants.DRIVER_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);

        // Initialize the dashboard selectors
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Box", AutoPattern.BOX);

        waitTimeChooser.setDefaultOption("No wait", 0);
        SmartDashboard.putData("Auto Wait Time", waitTimeChooser);
        waitTimeChooser.addOption("1 second", 1);
        waitTimeChooser.addOption("3 seconds", 3);
        waitTimeChooser.addOption("5 seconds", 5);

        driveModeChooser.setDefaultOption("Arcade", DriveMode.ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Tank", DriveMode.TANK);
        driveModeChooser.addOption("Single Stick (L)", DriveMode.SINGLE_STICK_LEFT);
        driveModeChooser.addOption("Single Stick (R)", DriveMode.SINGLE_STICK_RIGHT);
    }

    /**
     * Configure the button bindings for all operator commands
     * <p>
     * NOTE: This routine requires all subsystems to be passed in
     * <p>
     * NOTE: This routine must only be called once from the RobotContainer
     *
     * @param driveSubsystem
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem));

        // Gyro and Encoder Reset
        new Trigger(() -> driverController.getBackButton())
            .onTrue(new InstantCommand(() -> {
                driveSubsystem.resetGyro();
                driveSubsystem.resetEncoders();
            }));

        // Configure the DPAD to drive one meter on a heading
        new Trigger(() -> driverController.getPOV() == 0)
            .onTrue(new DriveOnHeadingCommand(0, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 90)
            .onTrue(new DriveOnHeadingCommand(90, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 180)
            .onTrue(new DriveOnHeadingCommand(180, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 270)
            .onTrue(new DriveOnHeadingCommand(270, .5, 100, driveSubsystem));
    }

    /*
     * Auto Pattern Selectors
     */
    public AutoPattern getAutoPattern() {
        return autoPatternChooser.getSelected();
    }

    public Integer getAutoDelay() {
        return waitTimeChooser.getSelected();
    }

    /*
     * Cancel Command support
     * Do not end the command while the button is pressed
     */
    public boolean isCancel() {
        return driverController.getStartButton();
    }

    /*
     * The following routines are used by the default commands for each subsystem
     *
     * They allow the default commands to get user input to manually move the
     * robot elements.
     */
    /*
     * Drive Subsystem
     */
    public DriveMode getSelectedDriveMode() {
        return driveModeChooser.getSelected();
    }

    public boolean isBoost() {
        return driverController.getLeftBumperButton();
    }

    public boolean isSlowDown() {
        return driverController.getRightBumperButton();
    }

    public double getLeftSpeed() {
        return driverController.getLeftY();
    }

    public double getRightSpeed() {
        return driverController.getRightY();
    }

    public double getSpeed() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_RIGHT) {
            return driverController.getRightY();
        }

        return driverController.getLeftY();
    }

    public double getTurn() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_LEFT) {
            return driverController.getLeftX();
        }

        return driverController.getRightX();
    }

    /*
     * Support for haptic feedback to the driver
     */
    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", driverController.toString());
    }

}
