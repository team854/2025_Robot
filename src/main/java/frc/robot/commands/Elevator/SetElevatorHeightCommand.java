package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            targetHeight;     // in feet for the top stage
    private final boolean           moveLowerUp;      // true for up, false for down

    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight, boolean moveLowerUp) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetHeight      = targetHeight;
        this.moveLowerUp       = moveLowerUp;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Set the upper stage to the target height
        int setpointIndex = findSetpointIndexForHeight(targetHeight);
        elevatorSubsystem.setUpperStagePosition(setpointIndex);

        // Move the lower stage up or down
        if (moveLowerUp) {
            elevatorSubsystem.moveLowerStageUp();
        }
        else {
            elevatorSubsystem.moveLowerStageDown();
        }
    }

    @Override
    public void execute() {
        // Continuously update the position of the top stage with PID
        elevatorSubsystem.setUpperStagePosition(findSetpointIndexForHeight(targetHeight));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopUpperStage();
        elevatorSubsystem.stopLowerStage();
    }

    @Override
    public boolean isFinished() {
        // Define when the command is finished.
        return elevatorSubsystem.getUpperStageEncoderPosition() >= targetHeight - Tolerances.ELEVATOR_LOWER_TOLERANCE &&
            elevatorSubsystem.getUpperStageEncoderPosition() <= targetHeight + Tolerances.ELEVATOR_UPPER_TOLERANCE;
    }

    private int findSetpointIndexForHeight(double height) {
        // This method maps the height to the appropriate setpoint index from your predefined setpoints
        for (int i = 0; i < ElevatorSubsystem.UPPER_STAGE_SETPOINTS.length; i++) {
            if (height <= ElevatorSubsystem.UPPER_STAGE_SETPOINTS[i]) {
                return i;
            }
        }
        return ElevatorSubsystem.UPPER_STAGE_SETPOINTS.length - 1; // Default to the highest setpoint
    }
}
