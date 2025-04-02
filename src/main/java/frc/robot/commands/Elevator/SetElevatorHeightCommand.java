package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            upperStageTargetHeight; // in feet
    private final double            lowerStageTargetHeight; // in feet

    public SetElevatorHeightCommand(ElevatorSubsystem elevatorSubsystem,
        double upperStageTargetHeight,
        double lowerStageTargetHeight) {
        this.elevatorSubsystem      = elevatorSubsystem;
        this.upperStageTargetHeight = upperStageTargetHeight;
        this.lowerStageTargetHeight = lowerStageTargetHeight;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Set the desired setpoints using the PID controllers in the subsystem
        elevatorSubsystem.setUpperStage(upperStageTargetHeight);
        elevatorSubsystem.setLowerStage(lowerStageTargetHeight);
        System.out.println("Initializing Elevator Setpoints: Upper = " + upperStageTargetHeight
            + ", Lower = " + lowerStageTargetHeight);
    }

    @Override
    public void execute() {
        // No need for additional control here since the PID loops are running in periodic()
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopUpperStage();
        elevatorSubsystem.stopLowerStage();
        System.out.println("Elevator command ended" + (interrupted ? " due to interruption" : ""));
    }

    @Override
    public boolean isFinished() {
        /*
         * Command finishes if within tolerance
         */
        return elevatorSubsystem.isUpperAtSetpoint() && elevatorSubsystem.isLowerAtSetpoint();
    }
}
