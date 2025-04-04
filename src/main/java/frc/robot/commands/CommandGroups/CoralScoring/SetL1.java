package frc.robot.commands.CommandGroups.CoralScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetArmAngleCommand;
import frc.robot.commands.Arm.SetWristPositionCommand;
import frc.robot.commands.Elevator.SetElevatorHeightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetL1 extends ParallelCommandGroup {

    public SetL1(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {

        addCommands(

            /*
             * Set elevator to L1 setpoint
             */
            new SetElevatorHeightCommand(elevatorSubsystem, ElevatorConstants.UPPER_ELEVATOR_L1_SETPOINT,
                ElevatorConstants.LOWER_ELEVATOR_L1_SETPOINT),

            /*
             * Set wrist to horizontal position
             */
            new SetWristPositionCommand(armSubsystem, ArmConstants.WRIST_HORIZONTAL_ANGLE),

            /*
             * Set arm to L1 angle
             */
            new SetArmAngleCommand(armSubsystem, ArmConstants.ARM_L1_ANGLE));
    }


}
