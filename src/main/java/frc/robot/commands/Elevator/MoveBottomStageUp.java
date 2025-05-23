package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveBottomStageUp extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            bottomStageSpeed; // Given in feet


    public MoveBottomStageUp(ElevatorSubsystem elevatorSubsystem, double bottomStageSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.bottomStageSpeed  = bottomStageSpeed;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setBottomStageSpeed(bottomStageSpeed);
        System.out.println("Moving Bottom Stage Up");
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isLowerStageAtUpperLimit();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopLowerStage();
    }

}
