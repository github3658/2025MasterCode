package frc.robot.commands.autonomous;

import frc.robot.Logger;

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
//region Testing
public class AutonomousPrograms {
    public static SequentialCommandGroup auto_Test1(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator) {
      return new SequentialCommandGroup(
        new DriveToPoseCommand(s_Swerve, s_Elevator, Position.PulloutTest),
        new DriveToPoseCommand(s_Swerve, s_Elevator, Position.PulloutTurnTest)
      );
    }
  
    public static SequentialCommandGroup auto_Test2(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator) {
      return new SequentialCommandGroup(
        new DriveToPoseCommand(s_Swerve, s_Elevator, Position.PulloutTurnTest)
        // new DriveToPoseCommand(s_Swerve, s_Elevator, Position.PulloutTurn)
      );
    }

    //endregion
    //region Right
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
          // new ParallelCommandGroup(
          //   new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
          //   new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStationBackup)
          // ),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStation),
            new ButtonPanelPressCommand(Button.CoralIn, true).repeatedly().until(() -> s_EndEffector.hasCoral())
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
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStation),
            new ButtonPanelPressCommand(Button.CoralIn, true).repeatedly().until(() -> s_EndEffector.hasCoral())
          )
          // new ParallelCommandGroup(
          //   new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
          //   new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ABackup)
          // ),
          // new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef8ABackupSpin)
          )
        );
    }
    //endregion
    //region Left
    public static ParallelCommandGroup auto_Left(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator, ButtonPanel bp_Operator, EndoFactorSubsystem s_EndEffector) {
      return new ParallelCommandGroup(
        new ParallelCommandGroup(
          new ElevatorDefaultCommand(s_Elevator, bp_Operator, s_EndEffector).ignoringDisable(true).withDeadline(null),
          new EndoFactorDefaultCommand(s_EndEffector, bp_Operator, s_Elevator).ignoringDisable(true)
        ),

        new SequentialCommandGroup (
          new ParallelCommandGroup(
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef11ACoral),
            new ButtonPanelPressCommand(Button.ElevatorPosition4, true)
          ), 
          new WaitForTrue(() -> s_Elevator.isFinished()),
          new WaitForTrue(() -> s_EndEffector.isFinished()), 
          new LogLocationCommand(s_Swerve, "DROP OFF CORAL 1"),
          new ButtonPanelPressCommand(Button.CoralOut, true),
          new WaitForDelay(0.1),
          // new ParallelCommandGroup(
          //   new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
          //   new DriveToPoseCommand(s_Swerve, s_Elevator, Position.CoralStationBackup)
          // ),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.invCoralStation),
            new ButtonPanelPressCommand(Button.CoralIn, true).repeatedly().until(() -> s_EndEffector.hasCoral())
          ),
          new LogLocationCommand(s_Swerve, "INTAKE CORAL"),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.ElevatorPosition4, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef12ACoral)
          ),
          new WaitForTrue(() -> s_Elevator.isFinished()),
          new WaitForTrue(() -> s_EndEffector.isFinished()),
          new ButtonPanelPressCommand(Button.CoralOut, true),
          new WaitForDelay(0.1),
          new ParallelCommandGroup(
            new WaitForDelay(0.5).andThen(new ButtonPanelPressCommand(Button.Stow, true)),
            new DriveToPoseCommand(s_Swerve, s_Elevator, Position.invCoralStation),
            new ButtonPanelPressCommand(Button.CoralIn, true).repeatedly().until(() -> s_EndEffector.hasCoral())
          )
        )
      );
    }
    //endregion
    //region Line
    public static SequentialCommandGroup auto_Line(SwerveDrivetrainSubsystem s_Swerve, ElevatorSubsystem s_Elevator) {
      return new SequentialCommandGroup(
        new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Pullout)
      );
    }
    //endregion
    //region Center
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
              new WaitForDelay(0.125),
              new ButtonPanelPressCommand(Button.CoralOut, true),
              new WaitForDelay(0.25),
              new ParallelCommandGroup(
                new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10ABackup),
                // Move to algae position
                new SequentialCommandGroup(
                  new WaitForDelay(0.75),
                  new ButtonPanelPressCommand(Button.ElevatorPosition3, true),
                  new ButtonPanelPressCommand(Button.ClawPositionAlgae, true),
                  new WaitForDelay(0.05)
                )
              ),
              new WaitForTrue(() -> s_Elevator.isFinished()),
              new WaitForTrue(() -> s_EndEffector.isFinished()),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10AlgaeBackup),
              // Intake algae
              new ButtonPanelPressCommand(Button.AlgaeIn, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10Algae).until(() -> s_EndEffector.likelyHasAlgae()),
              new WaitForTrue(() -> s_EndEffector.likelyHasAlgae()).withTimeout(3.5),
              new LogLocationCommand(s_Swerve, "GOT ALGAE"),
              // Go to processor
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Origin, 0.30),
              //new DriveToPoseCommand(s_Swerve, s_Elevator, Position.Reef10AlgaeBackup),
              new ButtonPanelPressCommand(Button.ElevatorPosition2, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.FaceProcessor),
              new ButtonPanelPressCommand(Button.AlgaeIn, false),
              new ButtonPanelPressCommand(Button.AlgaeOut, true),
              new WaitForDelay(1.0),
              new ButtonPanelPressCommand(Button.AlgaeOut, false),
              new WaitForDelay(0.05),
              new ButtonPanelPressCommand(Button.ClawPositionCoral, true),
              new ButtonPanelPressCommand(Button.Stow, true),
              new DriveToPoseCommand(s_Swerve, s_Elevator, Position.FleeFromProcessor)
            )
          );
    }
}
//endregion