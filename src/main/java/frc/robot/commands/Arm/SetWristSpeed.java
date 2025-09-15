package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetWristSpeed extends Command {
    private final ArmSubsystem armSubsystem;
    private final double       speed;


    public SetWristSpeed(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed        = speed;

    }

    @Override
    public void execute() {
        armSubsystem.setWristSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopWrist();
    }

}
