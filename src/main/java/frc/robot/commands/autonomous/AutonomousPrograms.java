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
    public static ParallelCommandGroup auto_Right(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator, ButtonPanel bp_Operator, EndoFactorSubsystem s_EndEffector) {
      return new ParallelCommandGroup(
        new ParallelCommandGroup(
          new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector).ignoringDisable(true).withDeadline(null),
          new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator).ignoringDisable(true)
        ),

        new SequentialCommandGroup (
          new ParallelCommandGroup(
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef9ACoral), 
            new ButtonPanelPressCommand(Button.ElevatorPosition4, true)
          ), 
          new WaitForTrue(() -> s_Elevator.isFinished()),
          new WaitForTrue(() -> s_EndEffector.isFinished()), 
          new LogLocationCommand(s_Swerve, "DROP OFF CORAL 1"),
          new ButtonPanelPressCommand(Button.CoralOut, true),
          new WaitForDelay(0.1),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStationBackup)
          ),
          new ParallelCommandGroup(
            new ButtonPanelPressCommand(Button.CoralIn, true).repeatedly().until(() -> s_EndEffector.hasCoral()),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStation)
          ),
          new LogLocationCommand(s_Swerve, "INTAKE CORAL"),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.ElevatorPosition4, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ACoral)
          ),
          new WaitForTrue(() -> s_Elevator.isFinished()),
          new WaitForTrue(() -> s_EndEffector.isFinished()),
          new ButtonPanelPressCommand(Button.CoralOut, true),
          new WaitForDelay(0.1),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ABackup)
          )
          // new ParallelCommandGroup(
          //   new SequentialCommandGroup(
          //     new WaitForDelay(0.1),
          //     new ButtonPanelPressCommand(Button.ElevatorPosition4, true)
          //   ),
          //   new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ACoral)
          // ),
          // new ButtonPanelPressCommand(Button.CoralOut, true),
          // new WaitForDelay(0.25),
          // new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ABackup),
          // new ButtonPanelPressCommand(Button.Stow, true)
            //   new ParallelCommandGroup(
            //     new ButtonPanelPressCommand(Button.Stow, true),
            //     new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStation)
            //   )
            )
        );
    }

    public static SequentialCommandGroup auto_Line(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator) {
      return new SequentialCommandGroup(
        new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Pullout)
      );
    }

    public static ParallelCommandGroup auto_Center(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator, ButtonPanel bp_Operator, EndoFactorSubsystem s_EndEffector) {
        return new ParallelCommandGroup(
            new ParallelCommandGroup(
              new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector).ignoringDisable(true).withDeadline(null),
              new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator).ignoringDisable(true)
            ),
            new SequentialCommandGroup(
              // Move to reef position
              new ButtonPanelPressCommand(Button.ElevatorPosition4, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10ACoral),
              new WaitForTrue(() -> s_Elevator.isFinished()),
              new WaitForTrue(() -> s_EndEffector.isFinished()),
              // Eject coral
              new LogLocationCommand(s_Swerve, "DROP OFF CORAL 1"),
              new ButtonPanelPressCommand(Button.CoralOut, true),
              new WaitForDelay(0.05),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10ABackup),
              // Move to algae position
              new ButtonPanelPressCommand(Button.ElevatorPosition3, true),
              new ButtonPanelPressCommand(Button.ClawPositionAlgae, true),
              new WaitForDelay(0.05),
              new WaitForTrue(() -> s_Elevator.isFinished()),
              new WaitForTrue(() -> s_EndEffector.isFinished()),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10AlgaeBackup),
              // Intake algae
              new ButtonPanelPressCommand(Button.AlgaeIn, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10Algae).until(() -> s_EndEffector.hasAlgae()),
              new LogLocationCommand(s_Swerve, "GOT ALGAE"),
              // Go to processor
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Origin),
              new ButtonPanelPressCommand(Button.ElevatorPosition2, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.FaceProcessor),
              new ButtonPanelPressCommand(Button.AlgaeIn, false),
              new ButtonPanelPressCommand(Button.AlgaeOut, true),
              new WaitForDelay(2.0),
              new ButtonPanelPressCommand(Button.AlgaeOut, false),
              new ButtonPanelPressCommand(Button.ClawPositionCoral, true),
              new ButtonPanelPressCommand(Button.Stow, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.FleeFromProcessor)
            )
          );
    }
}
