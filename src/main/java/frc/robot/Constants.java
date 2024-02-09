// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    // Climbing
    public static final int kClimbToSetpointButton = 2;// I HAVE NO CLUE WHICH BUTTON THIS IS LMAO FIX THIS
    public static final int kClimbUpButton = 3;
    public static final int kClimbDownButton = 4;

    // Intake
    public static final int kIntakeEnableMotorButton = 6;
    public static final int kIntakeDisableMotorButton = 7;
  }

  public static class FlywheelConstants {
    public static final double kMaxRPM = 5828;

    public static final int kFlywheelId = 5;

    public static final double kP = 6e-4;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIZone = 0;
    public static final double kFF = 1 / kMaxRPM;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
  }

  public static class ClimbingConstants {
    public static final int kLeftClimbMotorId = 6;
    public static final int kRightClimbMotorId = 7;

    public static final double climberVoltage = 12;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIZone = 0;
    public static final double kFF = 0; // TODO: BASE THIS ON THE ACTUAL GRAVITY FORCE OF ROBOT
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final double climbingDistance = 1.0;
    public static final double climbTolerance = (0.05) * climbingDistance;
    public static final double climbRateMax = 1.0;
    public static final int limitSwitchDigitalPort = 1;

    public static final double kOperatorClimbSpeed = 0.3;

    public static final double kPositionConversionFactor = 1.0; // depends on gear ratio and diameter

    public static final int kSmartCurrentLimit = 30;
  }

  public static class DriveConstants {
    public static final int frontLeftId = 1;
    public static final int frontRightId = 3;
    public static final int backLeftId = 2;
    public static final int backRightId = 4;

    public static final double DRIVE_GEAR_RATIO = 8.45;
    public static final double WHEEEL_CIRCUMFERENCE_INCHES = 18.875;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEEL_CIRCUMFERENCE_INCHES);

    public static final double limit = 1.0;

    public static final double FAST = 1.0;
    public static final double MEDIUM = 0.65;
    public static final double SLOW = 0.30;

    public static final double kPTurn = 0.01;
    public static final double kITurn = 0.0;
    public static final double kDTurn = 0.0;

    public static final double kPStraight = 2;
    public static final double kIStraight = 0.0;
    public static final double kDStraight = 0.0;

    public static final double trackWidthInches = 22;
    public static final double trackWidthMeters = Units.inchesToMeters(trackWidthInches);
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorId = 7;
    public static final int kIntakeArmId = 6;

    public static final int kSmartCurrentLimit = 30;

    public static final double kPositionConversionFactor = 1.0; // depends on gear ratio and diameter

    public static final double kIntakeSpeed = 0.3;

    public static final double kIntakeArmSpeed = 0.3;
    public static final double kIntakeArmDistance = 1.0;
    public static final double kIntakeArmTolerance = (0.05) * kIntakeArmDistance;
    public static final double kIntakeArmP = 1.0;
    public static final double kIntakeArmI = 0;
    public static final double kIntakeArmD = 0;
    public static final double kIntakeArmIZone = 0;
    public static final double kIntakeArmFF = 0;
    public static final double kIntakeArmMaxOutput = 1.0;
    public static final double kIntakeArmMinOutput = -1.0;
    public static final double kIntakeArmFFGravity = 0;

    public static final double intakeMotorSpeed = 0.5;
    public static final int limitSwitchDigitalPort = 0;

    public static final double intakePositionStowed = 0.0;
    public static final double intakePositionDeployed = 1.0;

  }

  public static class DebugSettings {
    public static final boolean enableLogging = false;
  }

}
