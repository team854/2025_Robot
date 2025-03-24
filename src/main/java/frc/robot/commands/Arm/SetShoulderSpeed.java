package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetShoulderSpeed extends Command {
    private final ArmSubsystem armSubsystem;
    private final double       speed;

    public SetShoulderSpeed(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed        = speed;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.setShoulderSpeed(speed);

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}