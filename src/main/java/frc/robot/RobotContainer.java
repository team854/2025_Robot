// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.CommandGroups.CoralIntake.GroundIntake;
import frc.robot.commands.CommandGroups.CoralIntake.SourceIntake;
import frc.robot.commands.CommandGroups.CoralScoring.ScoreCoral;
import frc.robot.commands.CommandGroups.CoralScoring.SetL1;
import frc.robot.commands.CommandGroups.CoralScoring.SetL2;
import frc.robot.commands.CommandGroups.CoralScoring.SetL3;
import frc.robot.commands.CommandGroups.CoralScoring.SetL4;
import frc.robot.commands.Elevator.MoveBottomStageDown;
import frc.robot.commands.Elevator.MoveTopStageDown;
import frc.robot.commands.Elevator.MoveTopStageUp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem       drivebase            = new SwerveSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ElevatorSubsystem     elevatorSubsystem    = new ElevatorSubsystem();
    private final ArmSubsystem          armSubsystem         = new ArmSubsystem();
    private final ClimbSubsystem        climbSubsystem       = new ClimbSubsystem();

    private final AutoModeChooser       autoModeChooser      = new AutoModeChooser(drivebase);

    /*
     * Command Groups
     */
    private final ScoreCoral            scoreCoralCommand;
    private final SetL1                 setL1Command;
    private final SetL2                 setL2Command;
    private final SetL3                 setL3Command;
    private final SetL4                 setL4Command;
    private final GroundIntake          groundIntakeCommand;
    private final SourceIntake          sourceIntakeCommand;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController   = new CommandXboxController(
        OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        /*
         * All commands
         */
        // Command groups
        scoreCoralCommand   = new ScoreCoral(elevatorSubsystem, armSubsystem);
        setL1Command        = new SetL1(elevatorSubsystem, armSubsystem);
        setL2Command        = new SetL2(elevatorSubsystem, armSubsystem);
        setL3Command        = new SetL3(elevatorSubsystem, armSubsystem);
        setL4Command        = new SetL4(elevatorSubsystem, armSubsystem);
        groundIntakeCommand = new GroundIntake(elevatorSubsystem, armSubsystem);
        sourceIntakeCommand = new SourceIntake(elevatorSubsystem, armSubsystem);

        // Base commands

        // Register Named Commands
        // configureNamedCommands();

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

        // --------------------------------------------------------
        // Driver Controller Commands
        // --------------------------------------------------------

        // // Score coral and lower arm and elevator (RT)
        // m_driverController.rightTrigger().onTrue(scoreCoralCommand);
        m_driverController.leftTrigger().whileTrue(new IntakeCommand(armSubsystem, true, ArmConstants.INTAKE_GROUND_SPEED));
        m_driverController.rightTrigger().whileTrue(new IntakeCommand(armSubsystem, false, ArmConstants.BRANCH_SCORE_SPEED));



        // // --------------------------------------------------------
        // // Operator Controller Commands
        // // --------------------------------------------------------

        // // Set elevator and arm to ground setpoint
        m_operatorController.a().onTrue(new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_GROUND_ANGLE));

        // // Set elevator and arm to horizontal setpoint
        m_operatorController.x().onTrue(new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_HORIZONTAL_ANGLE));

        // // Set elevator and arm to L4 setpoint
        m_operatorController.y().onTrue(new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L4_ANGLE));

        // // Set elevator and arm to top setpoint
        m_operatorController.b().onTrue(new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_TOP_ANGLE));

        // // Set elevator and arm to source intake setpoints, begin intaking (LB)
        // m_operatorController.leftBumper().onTrue(sourceIntakeCommand);

        // // Set elevator and arm to ground intake setpoints, begin intaking (LT)
        // m_operatorController.leftTrigger().onTrue(groundIntakeCommand);

        // Winch climb / raise robot (dpad up)
        m_operatorController.pov(0).whileTrue(new ClimbCommand(climbSubsystem,
            -ClimbConstants.CLIMB_UP_SPEED));

        // Unwinch climb / lower robot (dpad down)
        // m_operatorController.pov(180).whileTrue(new ClimbCommand(climbSubsystem,
        // ClimbConstants.CLIMB_DOWN_SPEED));

        m_operatorController.rightTrigger().whileTrue(new MoveTopStageDown(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_TOP_STAGE_DOWN_SPEED * -1));
        m_operatorController.rightBumper().whileTrue(new MoveTopStageUp(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_TOP_STAGE_UP_SPEED));
        m_operatorController.leftTrigger().whileTrue(new MoveBottomStageDown(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_BOTTOM_STAGE_DOWN_SPEED * -1));
        m_operatorController.leftBumper().whileTrue(new MoveTopStageUp(elevatorSubsystem,
            ElevatorConstants.ELEVATOR_BOTTOM_STAGE_UP_SPEED));

        // m_operatorController.b().whileTrue(new SetWristSpeed(armSubsystem, -0.2));
        // m_operatorController.x().whileTrue(new SetWristSpeed(armSubsystem, 0.2));
        //
        // m_operatorController.y().whileTrue(new SetShoulderSpeed(armSubsystem, 1));
        // m_operatorController.a().whileTrue(new SetShoulderSpeed(armSubsystem, -1));

    }

    /*
     * Methods used by arm default commands
     */
    public double getShoulderSpeed() {
        return -deadband(m_operatorController.getLeftY(), 0.2);
    }

    public double getWristSpeed() {
        return deadband(m_operatorController.getRightX(), 0.2);
    }

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
}