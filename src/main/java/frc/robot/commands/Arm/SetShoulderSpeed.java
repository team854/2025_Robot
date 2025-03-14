package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetShoulderSpeed extends Command {
    private final ArmSubsystem armSubsystem;
    private final double       speed;

    public SetShoulderSpeed(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed        = speed;

        // Declare subsystem dependencies
        // addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setShoulderSpeed(speed);
        System.out.println("Setting shoulder to: " + speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}