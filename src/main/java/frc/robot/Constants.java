// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class EmpiricalConstants { // not really sure how to name this
    public static final double kNEOStallTorque = 2.6;
    public static final double kNEOFreeSpeed = 5676;
    public static final double kInputVoltage = 12; // technically not an empirical constant but i need to put this
                                                   // somewhere
  }

  public static class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static class DriverButtonConstants {
      public static final int kDriveSpeedPreset1Button = 8;
      public static final int kDriveSpeedPreset2Button = 9;
    }

    public static class OperatorButtonConstants {
      // Flywheel
      public static final int kShootNoteButton = 1;
      public static final int kAimAtStageButton = 2;
      public static final int kAimAtTrapButton = 3;

      // Climbing
      public static final int kClimbUpButton = 4;
      public static final int kClimbDownButton = 5;

      // Intake
      public static final int kStartIntakeButton = 6;
      public static final int kStopIntakeButton = 7;
    }

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

    public static final double kClimbExtendedSetpoint = 1.0;
    public static final double kClimbRetractedSetpoint = 0.0;
    public static final double kClimbTolerance = (0.05) * kClimbRetractedSetpoint;
    public static final double climbRateMax = 1.0;
    public static final int kLimitSwitchDIOPort = 1;

    public static final double kOperatorClimbSpeed = 0.3;

    public static final double kPositionConversionFactor = 1.0; // depends on gear ratio and diameter

    public static final int kSmartCurrentLimit = 30;
  }

  public static class DriveConstants {
    // CAN IDs
    public static final int frontLeftId = 1;
    public static final int frontRightId = 3;
    public static final int backLeftId = 2;
    public static final int backRightId = 4;

    // Speed Presets
    public static final double kThrottlePreset1 = 0.5;
    public static final double kRotationPreset1 = 0.3;

    public static final double kThrottlePreset2 = 0.2;
    public static final double kRotationPreset2 = 0.2;

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

    // TODO: modify this value when we change the wheels and stuff because THIS
    // NUMBER WILL CHANGE
    public static final double kMaxVelocity = 5.4;

    public static final double kP = 9e-5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIZone = 0;
    public static final double kFF = 1 / kMaxVelocity;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

  }

  public static class IntakeConstants {
    public static final int kIntakeMotorId = 7;
    public static final int kIntakeArmId = 6;

    public static final int kSmartCurrentLimit = 30;

    public static final double kPositionConversionFactor = 1.0; // depends on gear ratio and diameter

    public static final double kIntakeSpeed = 0.3;

    public static final double kIntakeArmSpeed = 0.3;
    public static final double kIntakeArmTolerance = 0.1;
    public static final double kIntakeArmP = 1.0;
    public static final double kIntakeArmI = 0;
    public static final double kIntakeArmD = 0;
    public static final double kIntakeArmIZone = 0;
    public static final double kIntakeArmFF = 0;
    public static final double kIntakeArmMaxOutput = 1.0;
    public static final double kIntakeArmMinOutput = -1.0;
    public static final double kIntakeArmSetpointStow = 0.0;
    public static final double kIntakeArmSetpointDeploy = 1.0;

    public static final double kIntakeArmFFGravity = 0;
    public static final double kIntakeArmFFStatic = 0;
    public static final double kIntakeArmFFVelocity = 0;
    public static final double kIntakeArmFFAcceleration = 0;

    public static final double intakeMotorSpeed = 0.5;
    public static final int limitSwitchDigitalPort = 0;

    public static final double intakePositionStowed = 0.0;
    public static final double intakePositionDeployed = 1.0;

  }

  public static class DebugSettings {
    public static final boolean enableLogging = false;
  }
}
