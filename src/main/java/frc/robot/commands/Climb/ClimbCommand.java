package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    public final ClimbSubsystem climbSubsystem;
    public final double         setSpeed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double setSpeed) {
        this.climbSubsystem = climbSubsystem;
        this.setSpeed       = setSpeed;
    }

    @Override
    public void initialize() {
        climbSubsystem.setClimbSpeed(0);
    }

    @Override
    public void execute() {
        climbSubsystem.setClimbSpeed(setSpeed);
    }
}
