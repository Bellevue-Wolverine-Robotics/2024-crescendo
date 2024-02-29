// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BackupPathPlanner;
import frc.robot.commands.flywheel.FeederStartCommand;
import frc.robot.commands.flywheel.FlywheelAimIntakeReceiveCommand;
import frc.robot.commands.flywheel.FlywheelAimSpeakerCommand;
import frc.robot.commands.flywheel.FlywheelShootCommand;
import frc.robot.commands.intake.GetIntakeStatus;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Teleop {

  private Teleop() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command getShootAtSpeakerRoutine(FlywheelSubsystem flywheelSubsystem) {
    return new SequentialCommandGroup(
        new FlywheelAimSpeakerCommand(flywheelSubsystem),
        new InstantCommand(flywheelSubsystem::startShooter, flywheelSubsystem),
        new WaitCommand(1),
        new InstantCommand(flywheelSubsystem::startFeeder, flywheelSubsystem),
        new WaitCommand(0.5),
        new InstantCommand(flywheelSubsystem::stopShooter, flywheelSubsystem),
        new InstantCommand(flywheelSubsystem::stopFeeder, flywheelSubsystem));
  }

  public static Command getFullIntakeRoutine(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem) {
    double extendIntakeAngle = 30.0;
    double restIntakeAngle = 0.0;

    double flyWheelShoulderRecieveAngle = 0.0;
    double flyWheelElbowRecieveAngle = 0.0;

    double flyWheelShoulderShootAngle = 30.0;
    double flyWheelElbowShootAngle = 20.0;
    // idk if these are correct values

    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new StartIntakeCommand(intakeSubsystem),
            /*
             * AFTER INTAKE IS IN RECIEVING POSITION,
             * INTAKE WHEELS STARTS SPINNING
             */
            new FlywheelAimIntakeReceiveCommand(flywheelSubsystem)

        ),
        /*
         * AT THIS POINT INTAKE IS IN RECIEVING POSITION AND THE INTAKE IS SPINNNING
         * THE FLYWHEEL IS IN POSITION TO RECIEVE FROM INTAKE
         */
        new GetIntakeStatus(intakeSubsystem),
        /*
         * AT THIS POINT ARM HAS ACQUIRED NOTE(VERIFIED BY LIMIT SWITCH)
         */
        new StopIntakeCommand(intakeSubsystem),
        /*
         * AT THIS POINT THE INTAKE ARM IS AT REST POSITION AND IN POSITION FOR FLYWHEEL
         * INTAKE TO RECIEVE
         * INTAKE MOTOR WILL STOP BECAUSE IT HAS ACUIRED NOTE
         */

        // TODO: TURN THIS INTO A COMMAND THAT SPINS FOR LIKE HALF A SECOND
        new InstantCommand(
            () -> {
              flywheelSubsystem.startFeeder();
            }
        /*
         * FLYWHEEL IS ALREAY IN POSITION, FLYWHEEL DOING INTAKE
         */
        )

    );
  }
}
