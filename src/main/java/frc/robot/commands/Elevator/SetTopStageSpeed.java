package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetTopStageSpeed extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double            topStageSpeed;    // Given in feet


    public SetTopStageSpeed(ElevatorSubsystem elevatorSubsystem, double topStageSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.topStageSpeed     = topStageSpeed;

        // Declare subsystem dependencies
        // addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setTopStageSpeed(topStageSpeed);
        System.out.println("Moving Top Stage");
    }

    @Override
    public boolean isFinished() {
        if (topStageSpeed < 0) {
            return elevatorSubsystem.isUpperStageAtLowerLimit();
        }
        if (topStageSpeed > 0) {
            return elevatorSubsystem.isUpperStageAtUpperLimit();
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both stages when the command ends
        elevatorSubsystem.stopUpperStage();
    }

}
