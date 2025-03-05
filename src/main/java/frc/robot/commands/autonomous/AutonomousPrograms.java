package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonPanel;
import frc.robot.ButtonPanel.Button;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveToPoseCommand.Position;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndoFactorSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.EndoFactorDefaultCommand;

public class AutonomousPrograms {
    public static ParallelCommandGroup auto_Center(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator, ButtonPanel bp_Operator, EndoFactorSubsystem s_EndEffector) {
        return new ParallelCommandGroup(
            new ParallelCommandGroup(
              new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector).ignoringDisable(true).withDeadline(null),
              new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator).ignoringDisable(true)
            ),
            new SequentialCommandGroup(
              // Move to reef position
              new ButtonPanelPressCommand(Button.ElevatorPosition4, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Face1LeftCoral),
              new WaitForTrue(() -> s_Elevator.isFinished()),
              new WaitForTrue(() -> s_EndEffector.isFinished()),
              // Eject coral
              new ButtonPanelPressCommand(Button.CoralOut, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Face1Backup),
              // Move to algae position
              new ButtonPanelPressCommand(Button.ElevatorPosition3, true),
              new ButtonPanelPressCommand(Button.ClawPositionAlgae, true),
              new WaitForDelay(0.05),
              new WaitForTrue(() -> s_Elevator.isFinished()),
              new WaitForTrue(() -> s_EndEffector.isFinished()),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Algae1Backup),
              // Intake algae
              new ButtonPanelPressCommand(Button.AlgaeIn, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Algae1).until(() -> s_EndEffector.hasAlgae()),
              // Go to processor
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Origin),
              new ButtonPanelPressCommand(Button.ElevatorPosition2, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.FaceProcessor),
              new ButtonPanelPressCommand(Button.AlgaeIn, false),
              new ButtonPanelPressCommand(Button.AlgaeOut, true),
              new WaitForDelay(2.0),
              new ButtonPanelPressCommand(Button.AlgaeOut, false),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Origin)
            )
          );
    }
}
