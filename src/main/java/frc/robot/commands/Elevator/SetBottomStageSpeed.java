package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetBottomStageSpeed extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            bottomStageSpeed; // Given in feet


    public SetBottomStageSpeed(ElevatorSubsystem elevatorSubsystem, double bottomStageSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.bottomStageSpeed  = bottomStageSpeed;

        // Declare subsystem dependencies
        // addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setBottomStageSpeed(bottomStageSpeed);
        System.out.println("Moving Bottom Stage");
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopLowerStage();
    }

}
