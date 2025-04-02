package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyroCommand extends Command {
    public final SwerveSubsystem swerveSubsystem;

    public ZeroGyroCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        swerveSubsystem.zeroGyro();
        System.out.println("----------RESET GYRO TO ZERO----------");
    }
}
