package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends LoggingCommand {

    private final DriveSubsystem driveSubsystem;
    private final OperatorInput  operatorInput;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param operatorInput which contains the drive mode selector.
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.operatorInput  = operatorInput;
        this.driveSubsystem = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        // Get the selected drive mode
        DriveMode driveMode          = operatorInput.getSelectedDriveMode();

        // Calculate the drive scaling factor based on the boost mode and the slow mode.
        double    driveScalingFactor = DriveConstants.DRIVE_SCALING_NORMAL;

        if (operatorInput.isBoost()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_BOOST;
        }
        if (operatorInput.isSlowDown()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_SLOW;
        }

        // If this is a tank drive robot, then the left and right speeds are set from the
        // joystick values.
        if (driveMode == DriveMode.TANK) {

            double leftSpeed  = operatorInput.getLeftSpeed();
            double rightSpeed = operatorInput.getRightSpeed();

            setTankDriveMotorSpeeds(leftSpeed, rightSpeed, driveScalingFactor);

        }
        else {

            double speed = operatorInput.getSpeed();
            double turn  = operatorInput.getTurn();

            setArcadeDriveMotorSpeeds(speed, turn, driveScalingFactor);
        }

    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }


    /**
     * Set the motor speeds based on tank drive.
     *
     * @param leftSpeed value
     * @param rightSpeed value
     * @param driveScalingFactor
     */
    private void setTankDriveMotorSpeeds(double leftSpeed, double rightSpeed, double driveScalingFactor) {

        double speed = (leftSpeed + rightSpeed) / 2.0;
        double turn  = (leftSpeed - rightSpeed) / 2.0;

        setArcadeDriveMotorSpeeds(speed, turn, driveScalingFactor);
    }


    /**
     * Calculate the scaled arcade drive speeds from the passed in values. In arcade mode, the turn
     * is cut in half to help control the robot more consistently.
     *
     * @param speed
     * @param turn
     * @param driveScalingFactor
     */
    private void setArcadeDriveMotorSpeeds(double speed, double turn, double driveScalingFactor) {

        // Cut the spin in half because it will be applied to both sides.
        // Spinning at 1.0, should apply 0.5 to each side.
        turn = turn / 2.0;

        // Keep the turn, and reduce the forward speed where required to have the
        // maximum turn.
        if (Math.abs(speed) + Math.abs(turn) > 1.0) {
            speed = (1.0 - Math.abs(turn)) * Math.signum(speed);
        }

        double leftSpeed  = (speed + turn) * driveScalingFactor;
        double rightSpeed = (speed - turn) * driveScalingFactor;

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

}