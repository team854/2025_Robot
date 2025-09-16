// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutoModeChooser;
import frc.robot.commands.Arm.DefaultArmCommand;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Elevator.MoveBottomStageDown;
import frc.robot.commands.Elevator.MoveBottomStageUp;
import frc.robot.commands.Elevator.MoveTopStageDown;
import frc.robot.commands.Elevator.MoveTopStageUp;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // All subsystems go here
    private final SwerveSubsystem       drivebase            = new SwerveSubsystem();
    private final ElevatorSubsystem     elevatorSubsystem    = new ElevatorSubsystem();
    private final ArmSubsystem          armSubsystem         = new ArmSubsystem();
    private final ClimbSubsystem        climbSubsystem       = new ClimbSubsystem();

    // Jonathan's custom auto mode chooser
    private final AutoModeChooser       autoModeChooser      = new AutoModeChooser(drivebase);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController   = new CommandXboxController(
        OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        // ----------Set default drive command here----------\\
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        armSubsystem.setDefaultCommand(new DefaultArmCommand(this, armSubsystem));

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

        /*----------------------------------------------------------------
         * DRIVER CONTROLLER COMMANDS
         * ----------------------------------------------------------------
         */
        // Intake and Outtake
        m_driverController.leftTrigger().whileTrue(new IntakeCommand(armSubsystem, true, ArmConstants.INTAKE_GROUND_SPEED));
        m_driverController.rightTrigger().whileTrue(new IntakeCommand(armSubsystem, false, ArmConstants.BRANCH_SCORE_SPEED));

        // Zero gyro
        m_driverController.button(7).onTrue(new ZeroGyroCommand(drivebase));

        // ----------Manual Elevator Control----------\\
        // Top stage up
        m_driverController.y().whileTrue(new MoveTopStageUp(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_TOP_STAGE_UP_SPEED));
        // Top stage down
        m_driverController.b().whileTrue(new MoveTopStageDown(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_TOP_STAGE_DOWN_SPEED));
        // Bottom stage up
        m_driverController.x().whileTrue(new MoveBottomStageUp(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_BOTTOM_STAGE_UP_SPEED));
        // Bottom stage down
        m_driverController.a().whileTrue(new MoveBottomStageDown(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_BOTTOM_STAGE_DOWN_SPEED));

        // ----------Arm and wrist position presets----------\\
        // Ground Intake
        m_driverController.pov(180).onTrue(new ParallelCommandGroup(
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_GROUND_ANGLE),
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_HORIZONTAL_ANGLE)));
        // L4
        m_driverController.pov(0).onTrue(new ParallelCommandGroup(
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L4_ANGLE),
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_VERTICAL_ANGLE)));
        // Trough
        m_driverController.pov(90).onTrue(new ParallelCommandGroup(
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L1_ANGLE),
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_HORIZONTAL_ANGLE)));
        // L2 and L3
        m_driverController.pov(270).onTrue(new ParallelCommandGroup(
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L3_ANGLE),
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_VERTICAL_ANGLE)));


        /*----------------------------------------------------------------
         * OPERATOR CONTROLLER COMMANDS
         * ----------------------------------------------------------------
         */
        // Allow arm to be tucked inside robot (safety toggle)
        m_operatorController.button(8)
            .onTrue(new InstantCommand(() -> ((DefaultArmCommand) armSubsystem.getDefaultCommand()).toggleLowerLimit()));

        // Climb (dpad up)
        m_operatorController.pov(0).whileTrue(new ClimbCommand(climbSubsystem,
            ClimbConstants.CLIMB_UP_SPEED));

    }

    // Manual control of the arm angle (driver)
    public double getShoulderSpeed() {
        if (m_driverController.rightBumper().getAsBoolean()) {
            return 0.5;
        }
        if (m_driverController.leftBumper().getAsBoolean()) {
            return -0.5;
        }
        else
            return 0;
    }

    // Manual control of the wrist (operator)
    public double getWristSpeed() {
        return -deadband(m_operatorController.getRightX(), 0.2);
    }

    // Controller deadband
    public double deadband(double input, double deadband) {

        if (Math.abs(input) > deadband) {
            return (Math.abs(input) - deadband) / (1 - deadband) * Math.signum(input);
        }
        return 0;
    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoModeChooser.getSelectedAutoCommand();
    }

    // Zero gyro to match the drivers field relative perspective
    public void zeroGyro() {
        drivebase.zeroGyro();
        System.out.println("----------RESET GYRO TO ZERO----------");
    }
}