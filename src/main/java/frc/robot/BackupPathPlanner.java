package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TurnAbsoluteDegrees;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

class one949Parallel {
    private ParallelCommandGroup commands;

    public one949Parallel() {
        commands = new ParallelCommandGroup();
    }

    public void addCommands(Command newCommand) {
        commands.addCommands(newCommand);
    }

    public Command getFullParallel() {
        return commands;

    }

}

public class BackupPathPlanner {
    // DO NOT SWAP WITH POSE2d, proved program corectness based off of CustomePose
    public static class CustomPose {
        public CustomPose(double x, double y, double rad) {
            this.x = x;
            this.y = y;
            this.rad = rad;
        }

        double x;
        double y;
        double rad;
    }

    public static class OnePoint {
        public OnePoint(CustomPose pos, Command[] commands) {
            this.position = pos;
            this.doCommands = commands;
        }

        // struct
        CustomPose position;
        Command[] doCommands;
    }

    /*
     * stopPoints: all points the robot will pass excluding startpoint
     * These points are on a coordinate position where everything is relative to
     * (0,0 0 deg)
     * If robot starts executing command at (x, y), the entire path will be shifted
     * up x, right y.
     * 
     * 
     */

    SequentialCommandGroup finalCommand = new SequentialCommandGroup();

    public BackupPathPlanner(OnePoint[] points, DriveSubsystem driveSubsystem) {
        CustomPose prevPos = new CustomPose(0, 0, 0);
        // 1. Rotate until heading towards dest
        // 2. drive to dests
        // 3. correct direction

        for (OnePoint point : points) {
            double deltaX = point.position.x - prevPos.x;
            double deltaY = point.position.y - prevPos.y;
            double headingAngle = Math.atan(deltaY / deltaX);
            if (deltaX < 0) {
                headingAngle += Math.PI;
            }

            headingAngle += prevPos.rad;

            SequentialCommandGroup driveSeq = new SequentialCommandGroup();

            driveSeq.addCommands(new TurnAbsoluteDegrees(headingAngle, driveSubsystem));
            driveSeq.addCommands(new DriveStraight(driveSubsystem,
                    Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2))));
            driveSeq.addCommands(new TurnAbsoluteDegrees(point.position.rad - headingAngle, driveSubsystem));

            ParallelCommandGroup atPointRun = new ParallelCommandGroup();
            atPointRun.addCommands(driveSeq);
            for (Command externCommands : point.doCommands) {
                atPointRun.addCommands(externCommands);
            }
            finalCommand.addCommands(atPointRun);
            prevPos = point.position;
        }

    }

    public Command getCommand() {
        return finalCommand;
    }

}
