package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetWristPositionCommand extends Command {
    public final ArmSubsystem armSubsystem;
    public final boolean      toNinetyDegrees;

    public SetWristPositionCommand(ArmSubsystem armSubsystem, boolean toNinetyDegrees) {
        this.armSubsystem    = armSubsystem;
        this.toNinetyDegrees = toNinetyDegrees;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.moveWrist(false);
    }

    @Override
    public void execute() {
        armSubsystem.moveWrist(toNinetyDegrees);
    }

}
