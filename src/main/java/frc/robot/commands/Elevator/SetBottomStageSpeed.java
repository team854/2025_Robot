package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetBottomStageSpeed extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            bottomStageSpeed;


    public SetBottomStageSpeed(ElevatorSubsystem elevatorSubsystem, double bottomStageSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.bottomStageSpeed  = bottomStageSpeed;

    }

    @Override
    public void execute() {
        elevatorSubsystem.setBottomStageSpeed(bottomStageSpeed);
        System.out.println("Moving Bottom Stage");
    }

    @Override
    public boolean isFinished() {
        if (bottomStageSpeed < 0) {
            return elevatorSubsystem.isLowerStageAtLowerLimit();
        }
        if (bottomStageSpeed > 0) {
            return elevatorSubsystem.isLowerStageAtUpperLimit();
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopLowerStage();
    }

}
