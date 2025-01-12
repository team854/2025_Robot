// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    // Declarre the lighting subsystem first and pass it into the other subsystem
    // constructors so that they can indicate status information on the lights
    private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final DriveSubsystem  driveSubsystem  = new DriveSubsystem(lightsSubsystem);

    // Driver and operator controllers
    private final OperatorInput   operatorInput   = new OperatorInput();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, driveSubsystem));

        // Configure the button bindings - pass in all subsystems
        operatorInput.configureButtonBindings(driveSubsystem);

        // Add a trigger to flash the LEDs in sync with the
        // RSL light for 5 flashes when the robot is enabled
        // This can happen also if there is a brown-out of the RoboRIO.
        new Trigger(() -> RobotState.isEnabled())
            .onTrue(new InstantCommand(() -> lightsSubsystem.setRSLFlashCount(5)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AutoCommand(operatorInput, driveSubsystem);
    }
}
