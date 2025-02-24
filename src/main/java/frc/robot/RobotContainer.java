// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem       drivebase          = new SwerveSubsystem();
    private final VisionSubsystem       visionSubsystem    = new VisionSubsystem();
    private final ElevatorSubsystem     elevatorSubsystem  = new ElevatorSubsystem();
    private final ArmSubsystem          armSubsystem       = new ArmSubsystem();
    private final ClimbSubsystem        climbSubsystem     = new ClimbSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        // ----------Set default drive command here----------\\
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    }

    // -------------------------Swerve Drive Code-------------------------\\

    // Rotational velocity for drive base
    SwerveInputStream driveAngularVelocity              = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * -1,
        () -> m_driverController.getLeftX() * -1)
        .withControllerRotationAxis(
            () -> m_driverController.getRightX() * OperatorConstants.SWERVE_ROTATION_SCALE)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(OperatorConstants.SWERVE_TRANSLATION_SCALE)

        // Swerve perspective changes depending on which side of the field the driverstation is on
        .allianceRelativeControl(true);

    // Desired angle of rotation for drive base
    SwerveInputStream driveDirectAngle                  = driveAngularVelocity.copy().withControllerHeadingAxis(
        () -> m_driverController.getRightX() * OperatorConstants.SWERVE_ROTATION_SCALE,
        () -> m_driverController.getRightY() * OperatorConstants.SWERVE_ROTATION_SCALE)
        .headingWhile(true);

    // Drive Commands
    Command           driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);
    Command           driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    // -------------------------------------------------------------------\\

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {


    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("4x L4 Coral Auto");
    }
}