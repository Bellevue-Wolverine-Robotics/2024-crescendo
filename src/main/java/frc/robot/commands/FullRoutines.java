// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BackupPathPlanner;
import frc.robot.commands.flywheel.FlywheelAimIntakeReceiveCommand;
import frc.robot.commands.flywheel.FlywheelAimSpeakerCommand;
import frc.robot.commands.flywheel.FlywheelMoveToMakeSpaceForIntakeCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.GetIntakeStatus;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.commands.intake.StowIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class FullRoutines {

  private FullRoutines() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command getShootAtSpeakerRoutine(FlywheelSubsystem flywheelSubsystem) {
    return new SequentialCommandGroup(
        new FlywheelAimSpeakerCommand(flywheelSubsystem),
        new InstantCommand(flywheelSubsystem::startShooter, flywheelSubsystem),
        new WaitCommand(1),
        new InstantCommand(flywheelSubsystem::startFeeder, flywheelSubsystem),
        new WaitCommand(1),
        new InstantCommand(flywheelSubsystem::stopShooter, flywheelSubsystem),
        new InstantCommand(flywheelSubsystem::stopFeeder, flywheelSubsystem),
        new FlywheelMoveToMakeSpaceForIntakeCommand(flywheelSubsystem));
  }

  public static Command getFullIntakeRoutine(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem) {
    return new SequentialCommandGroup(
        new FlywheelMoveToMakeSpaceForIntakeCommand(flywheelSubsystem),
        new DeployIntakeCommand(intakeSubsystem),
        new StowIntakeCommand(intakeSubsystem),
        new FlywheelAimIntakeReceiveCommand(flywheelSubsystem),
        new FeedNoteFromIntakeToFlywheelCommand(intakeSubsystem, flywheelSubsystem));
  }
}
// new ParallelCommandGroup(
// new StartIntakeCommand(intakeSubsystem),
// /*
// * AFTER INTAKE IS IN RECIEVING POSITION,
// * INTAKE WHEELS STARTS SPINNING
// */
// new FlywheelAimIntakeReceiveCommand(flywheelSubsystem)

// ),
// /*
// * AT THIS POINT INTAKE IS IN RECIEVING POSITION AND THE INTAKE IS SPINNNING
// * THE FLYWHEEL IS IN POSITION TO RECIEVE FROM INTAKE
// */
// new GetIntakeStatus(intakeSubsystem),
// /*
// * AT THIS POINT ARM HAS ACQUIRED NOTE(VERIFIED BY LIMIT SWITCH)
// */
// new StopIntakeCommand(intakeSubsystem),
// /*
// * AT THIS POINT THE INTAKE ARM IS AT REST POSITION AND IN POSITION FOR
// FLYWHEEL
// * INTAKE TO RECIEVE
// * INTAKE MOTOR WILL STOP BECAUSE IT HAS ACUIRED NOTE
// */

// // TODO: TURN THIS INTO A COMMAND THAT SPINS FOR LIKE HALF A SECOND
// new InstantCommand(
// () -> {
// flywheelSubsystem.startFeeder();
// }
// /*
// * FLYWHEEL IS ALREAY IN POSITION, FLYWHEEL DOING INTAKE
// */
// )
