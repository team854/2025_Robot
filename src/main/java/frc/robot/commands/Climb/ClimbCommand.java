package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    public final ClimbSubsystem climbSubsystem;
    public double               setSpeed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double setSpeed) {
        this.climbSubsystem = climbSubsystem;
        this.setSpeed       = setSpeed;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climbSubsystem.setClimbSpeed(setSpeed);
        System.out.println("Climbing");
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.setClimbSpeed(0);
    }
}
