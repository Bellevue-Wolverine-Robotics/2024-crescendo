// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Enums.AutoEnum;
import frc.robot.Enums.Throttles;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Debug debugLogger = new Debug("DebugDriveSubsystem.txt");
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(debugLogger);
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  private final IntakeMotorSubsystem m_intakeMotorSubsystem = new IntakeMotorSubsystem();

  private final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    configureBindings();
    smartDashBoardBinding();

  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(
        new ArcadeDrive(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX()));

    m_flywheelSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_flywheelSubsystem.setFlywheelVelocity(m_operatorController.getY() *
                FlywheelConstants.kMaxRPM),
            m_flywheelSubsystem));

    // climber
    m_operatorController.button(OperatorConstants.kClimbToSetpointButton)
        .onTrue(m_climberSubsystem.climbToPositionSetpointCommand(25));
    m_operatorController.button(OperatorConstants.kClimbUpButton)
        .whileTrue(m_climberSubsystem.climbUpCommand());
    m_operatorController.button(OperatorConstants.kClimbDownButton)
        .whileTrue(m_climberSubsystem.climbDownCommand());

    // intake
    // m_operatorController.button(OperatorConstants.kIntakeEnableMotorButton)
    // .onTrue(Autos.IntakeSequence(m_intakeArmSubsystem, m_intakeMotorSubsystem));
  }

  public void smartDashBoardBinding() {
    SmartDashboard.putData("Save logging info", new DebugClose(debugLogger));
  }

  public void setDriveThrottleSpeed(Throttles throttleSpeed) {
    switch (throttleSpeed) {
      case FAST:
        m_driveSubsystem.setSpeedLimit(DriveConstants.FAST);
        break;
      case MEDIUM:
        m_driveSubsystem.setSpeedLimit(DriveConstants.MEDIUM);
        break;
      case SLOW:
        m_driveSubsystem.setSpeedLimit(DriveConstants.SLOW);
        break;
      default:
        break;
    }

  }

  public Command getAutonomousCommand(AutoEnum autoEnum) {
    // return Autos.getPathPlannerCommand();
    switch (autoEnum) {
      case FOWARD_TEST:
        return Autos.forwardTest(m_driveSubsystem);
      case CUSTOM_PATH_PLANNER:
        return Autos.test949PathPlan(m_driveSubsystem);
      case PATH_PLANNER:
        return Autos.getPathPlannerCommand();
      default:
        return null;
    }
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }

}
