package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            targetHeightFeet; // Given in feet
    private final boolean           moveLowerUp;

    private double                  targetRotations;  // Converted to encoder units

    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, double targetHeightFeet, boolean moveLowerUp) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetHeightFeet  = targetHeightFeet;
        this.moveLowerUp       = moveLowerUp;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Convert feet to encoder rotations
        targetRotations = elevatorSubsystem.feetToRotations(targetHeightFeet);

        // Move the upper stage directly to the target height
        elevatorSubsystem.setUpperStageHeight(targetRotations);

        // Move the lower stage
        if (moveLowerUp) {
            elevatorSubsystem.moveLowerStageUp();
        }
        else {
            elevatorSubsystem.moveLowerStageDown();
        }
    }

    @Override
    public void execute() {
        // Continuously ensure the elevator moves towards the setpoint
        elevatorSubsystem.setUpperStageHeight(targetRotations);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopUpperStage();
        elevatorSubsystem.stopLowerStage();
    }

    @Override
    public boolean isFinished() {
        // Get the current encoder position
        double currentPosition = elevatorSubsystem.getUpperStageEncoderPosition();

        // Check if the elevator is within the acceptable range
        return Math.abs(currentPosition - targetRotations) <= Tolerances.ELEVATOR_UPPER_TOLERANCE;
    }
}
