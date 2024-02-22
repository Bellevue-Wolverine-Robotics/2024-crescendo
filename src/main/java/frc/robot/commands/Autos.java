// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BackupPathPlanner;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.drivetrain.DriveStraight;
import frc.robot.commands.intake.SetArmAngleCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command IntakeSequence(IntakeSubsystem m_intakeArmSubsystem,
      IntakeSubsystem intakeMotorSubsystem) {
    return null;

    // return new SequentialCommandGroup(new IntakeArm(m_intakeArmSubsystem,
    // IntakeConstants.intakePositionDeployed),
    // new IntakeMotor(intakeMotorSubsystem),
    // new IntakeArm(m_intakeArmSubsystem, IntakeConstants.intakePositionStowed));

    // return new SequentialCommandGroup(m_intakeArmSubsystem.goToAngle(1),
    // new IntakeMotor(intakeMotorSubsystem),
    // m_intakeArmSubsystem.goToAngle(0));
  }

  public static Command pathfindToStartCommand() {
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(0, 8, Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );

    return pathfindingCommand;
  }

  public static Command square300InchesCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Square300Inches");

    return AutoBuilder.followPath(path);
  }

  public static Command straight300InchesCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Straight300Inches");

    return AutoBuilder.followPath(path);
  }

  public static Command getPathPlannerCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");

    return AutoBuilder.followPath(path);
  }

  public static Command forwardTest(DriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        new DriveStraight(driveSubsystem, null, 5),
        new WaitCommand(2),
        new TurnRelative(1, driveSubsystem)

    );
  }

  public static Command test949PathPlan(DriveSubsystem driveSubsystem) {
    BackupPathPlanner.OnePoint[] poseList = new BackupPathPlanner.OnePoint[3];
    Command[] pCommands = {};
    poseList[0] = new BackupPathPlanner.OnePoint(new BackupPathPlanner.CustomPose(2.0, 0.0, 0.0), pCommands);
    poseList[1] = new BackupPathPlanner.OnePoint(new BackupPathPlanner.CustomPose(2.0, 4.0, 0.0), pCommands);
    poseList[2] = new BackupPathPlanner.OnePoint(new BackupPathPlanner.CustomPose(0.0, 0.0, 0.0), pCommands);

    BackupPathPlanner plan = new BackupPathPlanner(poseList, driveSubsystem);
    return plan.getCommand();
  }
}
