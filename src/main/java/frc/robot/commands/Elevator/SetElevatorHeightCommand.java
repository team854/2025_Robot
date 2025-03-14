package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            upperStageTargetHeight; // Given in feet
    private final double            lowerStageTargetHeight;


    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem, double upperStageTargetHeight,
        double lowerStageTargetHeight) {
        this.elevatorSubsystem      = elevatorSubsystem;
        this.upperStageTargetHeight = upperStageTargetHeight;
        this.lowerStageTargetHeight = lowerStageTargetHeight;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setUpperStage(upperStageTargetHeight);
        System.out.println("Setting upper stage to: " + upperStageTargetHeight);
        elevatorSubsystem.setLowerStage(lowerStageTargetHeight);
        System.out.println("Setting lower stage to: " + lowerStageTargetHeight);

    }

    @Override
    public void execute() {
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

        if (Math.abs(currentUpperStageHeight - upperStageTargetHeight) <= Tolerances.ELEVATOR_UPPER_TOLERANCE
            && (Math.abs(currentLowerStageHeight - lowerStageTargetHeight) <= Tolerances.ELEVATOR_UPPER_TOLERANCE)) {
            return true;
        }
        else {
            return false;
        }
    }
}
