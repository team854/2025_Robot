package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final boolean      isReversed;
    private final double       intakeSpeed;

    public IntakeCommand(ArmSubsystem armSubsystem, boolean isReversed, double intakeSpeed) {
        this.armSubsystem = armSubsystem;
        this.intakeSpeed  = intakeSpeed;
        this.isReversed   = isReversed;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        if (!armSubsystem.hasGamePiece()) {
            armSubsystem.setIntakeSpeed(intakeSpeed, isReversed);
        }
        else {
            armSubsystem.setIntakeSpeed(0, false); // Stop intake if a game piece is detected
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setIntakeSpeed(0, false);
    }
}
