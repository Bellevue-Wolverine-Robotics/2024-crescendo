// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drive;
import frc.robot.Constants.Flywheel;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Enums.Throttles;
import frc.robot.commands.*;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import pabeles.concurrency.IntOperatorTask.Max;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.apache.commons.io.input.MessageDigestCalculatingInputStream;

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

  private final CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  private Throttles prevThrottle;

  private SendableChooser<Throttles> throttleSelection;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(
        new ArcadeDrive(m_driveSubsystem, () -> m_driverController.getY(), () -> m_driverController.getX()));

    m_flywheelSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_flywheelSubsystem.setFlywheelVelocity(m_operatorController.getY() * Flywheel.kMaxRPM), // TODO:
            // change
            // to
            // constant
            m_flywheelSubsystem));

  }

  public void setDriveThrottleSpeed(Throttles throttleSpeed){
    switch (throttleSpeed){
      case FAST:
        m_driveSubsystem.setSpeedLimit(Drive.FAST);
        break;
      case MEDIUM:
        m_driveSubsystem.setSpeedLimit(Drive.MEDIUM);
        break;
      case SLOW:
        m_driveSubsystem.setSpeedLimit(Drive.SLOW);
        break;
      default:
        break;
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new DriveStraight(m_driveSubsystem, debugLogger);
    // An example command will be run in autonomous
  }
}
