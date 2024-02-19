// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.DriverButtonConstants;
import frc.robot.Constants.IOConstants.OperatorButtonConstants;
import frc.robot.Enums.AutoEnum;
import frc.robot.Enums.Throttles;
import frc.robot.commands.Autos;
import frc.robot.commands.DebugClose;
import frc.robot.commands.climber.ClimberExtendCommand;
import frc.robot.commands.climber.ClimberRetractCommand;
import frc.robot.commands.drivetrain.ArcadeDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandJoystick m_driverController = new CommandJoystick(IOConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(IOConstants.kOperatorControllerPort);

  public RobotContainer() {
    configureBindings();
    smartDashBoardBinding();
  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(
        new ArcadeDriveCommand(m_driveSubsystem, () -> getArcadeDriveSpeeds().getFirst(),
            () -> getArcadeDriveSpeeds().getSecond()));

    m_flywheelSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_flywheelSubsystem.setFlywheelVelocity(m_operatorController.getY() *
                FlywheelConstants.kMaxRPM),
            m_flywheelSubsystem));

    // climber
    m_operatorController.button(OperatorButtonConstants.kClimbUpButton)
        .whileTrue(new ClimberExtendCommand(m_climberSubsystem));
    m_operatorController.button(OperatorButtonConstants.kClimbDownButton)
        .whileTrue(new ClimberRetractCommand(m_climberSubsystem));

    // intake
    // m_operatorController.button(OperatorConstants.kIntakeEnableMotorButton)
    // .onTrue(Autos.IntakeSequence(m_intakeArmSubsystem, m_intakeMotorSubsystem));
  }

  public void smartDashBoardBinding() {
    SmartDashboard.putData("Save logging info", new DebugClose(debugLogger));
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

  /**
   * Returns arcade drive speeds based on throttles and squaring selected
   * 
   * @return A pair of the x speed and the z rotation
   */
  private Pair<Double, Double> getArcadeDriveSpeeds() {
    double xSpeed = -m_driverController.getY();
    double zRotation = -m_driverController.getX();

    if (DriverStation.getStickButton(IOConstants.kDriverControllerPort,
        DriverButtonConstants.kDriveSpeedPreset1Button)) {
      xSpeed *= DriveConstants.kThrottlePreset1;
      zRotation *= DriveConstants.kRotationPreset1;
    } else if (DriverStation.getStickButton(IOConstants.kDriverControllerPort,
        DriverButtonConstants.kDriveSpeedPreset2Button)) {
      xSpeed *= DriveConstants.kThrottlePreset2;
      zRotation *= DriveConstants.kRotationPreset2;
    }

    Pair<Double, Double> arcadeDriveSpeedsPair = new Pair<Double, Double>(xSpeed, zRotation);

    return arcadeDriveSpeedsPair;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_driveSubsystem;
  }
}
