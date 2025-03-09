package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            upperStageTargetHeightFeet; // Given in feet
    private final double            lowerStageTargetHeightFeet;


    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, double upperStageTargetHeightFeet,
        double lowerStageTargetHeightFeet) {
        this.elevatorSubsystem          = elevatorSubsystem;
        this.upperStageTargetHeightFeet = upperStageTargetHeightFeet;
        this.lowerStageTargetHeightFeet = lowerStageTargetHeightFeet;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Continuously ensure the elevator moves towards the setpoint
        elevatorSubsystem.setUpperStage(upperStageTargetHeightFeet);
        elevatorSubsystem.setLowerStage(lowerStageTargetHeightFeet);
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
        double currentUpperStageHeight = elevatorSubsystem.getUpperStageHeight();
        double currentLowerStageHeight = elevatorSubsystem.getLowerStageHeight();

        if (Math.abs(currentUpperStageHeight - upperStageTargetHeightFeet) <= Tolerances.ELEVATOR_UPPER_TOLERANCE
            && (Math.abs(currentLowerStageHeight - lowerStageTargetHeightFeet) <= Tolerances.ELEVATOR_UPPER_TOLERANCE)) {
            return true;
        }
        else {
            return false;
        }
    }
}
