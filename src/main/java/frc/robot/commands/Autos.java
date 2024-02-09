// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.commands.IntakeArm;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command IntakeSequence(IntakeArmSubsystem m_intakeArmSubsystem,
      IntakeMotorSubsystem intakeMotorSubsystem) {
    return null;

    // return new SequentialCommandGroup(new IntakeArm(m_intakeArmSubsystem,
    // IntakeConstants.intakePositionDeployed),
    // new IntakeMotor(intakeMotorSubsystem),
    // new IntakeArm(m_intakeArmSubsystem, IntakeConstants.intakePositionStowed));

    // return new SequentialCommandGroup(m_intakeArmSubsystem.goToAngle(1),
    // new IntakeMotor(intakeMotorSubsystem),
    // m_intakeArmSubsystem.goToAngle(0));
  }

  public static Command getPathPlannerCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("path101");

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  }

  public static Command forwardTest(DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        new DriveStraight(driveSubsystem, null, 5),
        new WaitCommand(2),
        new TurnAbsoluteDegrees(360, driveSubsystem)

    );
  }

}
