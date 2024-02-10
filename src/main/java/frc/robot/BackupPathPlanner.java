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
import frc.robot.commands.TurnRelative;
import frc.robot.commands.TurnRelativeRadians;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

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

    private double convertToCoordinateSystem(double angleRadians)
    //ensures valid angle 
    //\result < 2*Math.PI && \result <= 2*Math.PI
    {
        if(angleRadians < 0){
            angleRadians = Math.ceil(angleRadians/(2*Math.PI))*(2*Math.PI) + angleRadians;
        }
        angleRadians -= ((int)(angleRadians/(2*Math.PI)))*(2*Math.PI);
        return angleRadians;
    }
    
    private double optomizeTurnAngle(double angleRadians)
    //requires -2PI <= \result && \result <= 2PI
    //ensures -PI <= \result && \result <= PI
    {
        if(angleRadians > Math.PI){
            //@assert angleToTurn > 0;
            angleRadians -= 2*Math.PI;
        }
        if(angleRadians < -Math.PI){
            //@assert angleToTurn < 0;
            angleRadians += 2*Math.PI;
        }
        return angleRadians;
        
    }



    /*
     * stopPoints: all points the robot will pass excluding startpoint
     * These points are on a coordinate position where everything is relative to
     * (0,0 0 rad)
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

        /*STRICT COORDINATE SYSTEM, similar to unit circle
        The robot initially points towards 0 radians, +X direction
        Robot angular position/heading strictly less than 2π radians; Angular position resets at 2π to 0 radians    
        */

        for (OnePoint point : points) { 
            point.position.rad = convertToCoordinateSystem(point.position.rad);
            //Point is now valid in struct coordinate system.

            double deltaX = point.position.x - prevPos.x;
            double deltaY = point.position.y - prevPos.y;
            double headingAngle = Math.atan(deltaY / deltaX);
            if (deltaX < 0) {
                headingAngle += Math.PI;
            } 
            else if(headingAngle < 0){
                headingAngle = 2*Math.PI + headingAngle;
            }
            //headingAngle is strictly following coordinate system at this point.

            //@assert 0 < headingAngle < 2π
            //@assert 0 < prevPos.rad < 2π
            double angleToTurn = headingAngle - prevPos.rad;
            //@assert -2π < angleToTurn < 2π

            angleToTurn = optomizeTurnAngle(angleToTurn);
            //angleToTurn is optomized at this point.
            SequentialCommandGroup driveSeq = new SequentialCommandGroup();

            driveSeq.addCommands(new TurnRelative(angleToTurn, driveSubsystem));
            driveSeq.addCommands(new DriveStraight(driveSubsystem,
                    Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2))));

            double correctHeading = point.position.rad - headingAngle;
            correctHeading = optomizeTurnAngle(correctHeading);
            //correctHeading is optomized at this point.
            
            driveSeq.addCommands(
                    new TurnRelative(correctHeading, driveSubsystem));


            ParallelCommandGroup atPointRun = new ParallelCommandGroup();
            atPointRun.addCommands(driveSeq);

            for(Command externCommands: point.doCommands){
                if(externCommands != null){
                    atPointRun.addCommands(externCommands);
                }
                //else should assert
            }
            finalCommand.addCommands(atPointRun);
            prevPos = point.position; 
        }

    }

    public Command getCommand() {
        return finalCommand;
    }

}
