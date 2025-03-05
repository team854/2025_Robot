package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            targetHeightFeet;          // Given in feet
    private final double            lowerStageTargetHeightFeet;

    private double                  targetRotations;           // Converted to encoder units
    private double                  lowerStageTargetRotations;

    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, double targetHeightFeet,
        double lowerStageTargetHeightFeet) {
        this.elevatorSubsystem          = elevatorSubsystem;
        this.targetHeightFeet           = targetHeightFeet;
        this.lowerStageTargetHeightFeet = lowerStageTargetHeightFeet;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Convert feet to encoder rotations
        targetRotations           = elevatorSubsystem.upperfeetToRotations(targetHeightFeet);
        lowerStageTargetRotations = elevatorSubsystem.lowerfeetToRotations(targetHeightFeet);

        // Move the upper stage directly to the target height
        elevatorSubsystem.setUpperStage(targetRotations);
        elevatorSubsystem.setLowerStage(lowerStageTargetRotations);

    }

    @Override
    public void execute() {
        // Continuously ensure the elevator moves towards the setpoint
        elevatorSubsystem.setUpperStage(targetRotations);
        elevatorSubsystem.setLowerStage(lowerStageTargetRotations);
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
