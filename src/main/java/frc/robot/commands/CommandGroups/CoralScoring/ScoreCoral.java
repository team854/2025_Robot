package frc.robot.commands.CommandGroups.CoralScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.IntakeCommand;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ScoreCoral extends SequentialCommandGroup {

    public ScoreCoral(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {

        addCommands(

            /*
             * Reverse the intake so the coral releases easier
             */
            new IntakeCommand(armSubsystem, false, ArmConstants.BRANCH_SCORE_SPEED),

            /*
             * Set arm to L1 angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_DEFAULT_ANGLE),

            /*
             * Set elevator to default setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_DEFAULT_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_DEFAULT_SETPOINT));



    }


}
