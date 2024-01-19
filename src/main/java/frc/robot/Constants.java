// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.lang3.ObjectUtils.Null;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

/*
 * Constants -> properties -> subsystem or 
 * Constants -> subsystem -> properties
 * ahhhhhhhh
 * 
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CANConstants {
    public static final int frontLeftId = 1;
    public static final int frontRightId = 3;
    public static final int backLeftId = 2;
    public static final int backRightId = 4;

    public static final int kFlywheelId = 5;

    public static final int climbMotor1 = -1;
    public static final int climbMotor2 = -1; 
  }

  public static class PIDConstants {
    public static class FlywheelPID {
      public static final double kMaxRPM = 5828;

      public static final double kP = 6e-4;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIZone = 0;
      public static final double kFF = 1 / kMaxRPM;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
    }
  }

  public static class Throttles {
    public static final double limit = 1.0;
  }

  public static class Climbing{
    public static final double climberVoltage = 12;
    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double climbingDistance = 1.0;
    public static final double climbTolerance = (0.05)*climbingDistance;
    public static final double climbRateMax  = 1.0;
  }

  public static class DebugSettings {
    public static final boolean enableLogging = false;
  }

  public static class PhysicalConstants {
    public static final double DRIVE_GEAR_RATIO = 8.45;
    public static final double WHEEEL_CIRCUMFERENCE_INCHES = 18.875;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEEL_CIRCUMFERENCE_INCHES);
  }





}
