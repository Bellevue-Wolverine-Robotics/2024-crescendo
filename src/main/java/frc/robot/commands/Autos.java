// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BackupPathPlanner;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
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
