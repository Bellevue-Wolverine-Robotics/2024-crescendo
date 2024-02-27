// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DebugClose;
import frc.robot.commands.SubstituteCommand;
import frc.robot.commands.climber.ClimberContinuousDutyCommand;
import frc.robot.commands.climber.ClimberExtendCommand;
import frc.robot.commands.climber.ClimberResetCommand;
import frc.robot.commands.climber.ClimberRetractCommand;
import frc.robot.commands.drivetrain.ArcadeDriveCommand;
import frc.robot.commands.intake.GetFullIntakeRoutine;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.commands.intake.GetFullIntakeRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants.DriverButtonConstants;
import frc.robot.constants.IOConstants.JoystickPortConstants;
import frc.robot.constants.IOConstants.OperatorButtonConstants;
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
  private final SendableChooser<Command> m_autoChooser;

  private final Debug debugLogger = new Debug("DebugDriveSubsystem.txt");

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(debugLogger);
  // private final FlywheelSubsystem m_flywheelSubsystem = new
  // FlywheelSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final FlywheelSubsystem m_flyWheelSubsystem = new FlywheelSubsystem();

  private final CommandJoystick m_driverController = new CommandJoystick(JoystickPortConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(
      JoystickPortConstants.kOperatorControllerPort);

  public RobotContainer() {
    configureBindings();
    smartDashBoardBinding();

    NamedCommands.registerCommand("startIntake", new StartIntakeCommand(m_intakeSubsystem));
    NamedCommands.registerCommand("climberExtend", new ClimberExtendCommand(m_climberSubsystem));
    NamedCommands.registerCommand("climberRetract", new ClimberRetractCommand(m_climberSubsystem));
    NamedCommands.registerCommand("passNoteAndShoot", new SubstituteCommand());
    NamedCommands.registerCommand("print",
        new InstantCommand(() -> System.out.println("osuheotnsuheohuenohucrohucreohu")));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(
        new ArcadeDriveCommand(m_driveSubsystem, () -> getArcadeDriveSpeeds().getFirst(),
            () -> getArcadeDriveSpeeds().getSecond()));

    m_climberSubsystem.setDefaultCommand(
        new ClimberContinuousDutyCommand(m_climberSubsystem, m_operatorController));

    // climber
    m_operatorController.button(OperatorButtonConstants.kClimbUpButton)
        .whileTrue(new ClimberExtendCommand(m_climberSubsystem));
    m_operatorController.button(OperatorButtonConstants.kClimbDownButton)
        .whileTrue(new ClimberRetractCommand(m_climberSubsystem));
    m_operatorController.button(OperatorButtonConstants.kfullIntakeCycle).onTrue(GetFullIntakeRoutine.fullIntakeSequence(m_intakeSubsystem, m_flyWheelSubsystem));


    // intake
    // m_operatorController.button(OperatorConstants.kIntakeEnableMotorButton)
    // .onTrue(Autos.IntakeSequence(m_intakeArmSubsystem, m_intakeMotorSubsystem));

  }

  public void smartDashBoardBinding() {
    SmartDashboard.putData("Save logging info", new DebugClose(debugLogger));
  }

  public Command getAutonomousCommand() {
    // return new ParallelCommandGroup(new ClimberResetCommand(m_climberSubsystem));

    // return new ParallelCommandGroup(m_autoChooser.getSelected());
    return m_autoChooser.getSelected();

    // PathPlannerPath path = PathPlannerPath.fromPathFile("straight");
    // return AutoBuilder.followPath(path);

    // return Autos.getPathPlannerCommand();
    // switch (autoEnum) {
    // // case FOWARD_TEST:
    // // return Autos.forwardTest(m_driveSubsystem);
    // // case CUSTOM_PATH_PLANNER:
    // // return Autos.test949PathPlan(m_driveSubsystem);
    // default:
    // return null;
    // }
  }

  /**
   * Returns arcade drive speeds based on throttles and squaring selected
   * 
   * @return A pair of the x speed and the z rotation
   */
  private Pair<Double, Double> getArcadeDriveSpeeds() {
    double xSpeed = -m_driverController.getY();
    double zRotation = -m_driverController.getX();

    if (DriverStation.getStickButton(JoystickPortConstants.kDriverControllerPort,
        DriverButtonConstants.kDriveSpeedPreset1Button)) {
      xSpeed *= DriveConstants.kThrottlePreset1;
      zRotation *= DriveConstants.kRotationPreset1;
    } else if (DriverStation.getStickButton(JoystickPortConstants.kDriverControllerPort,
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

  public ClimberSubsystem getClimberSubsystem() {
    return m_climberSubsystem;
  }

  public double getStickY() {
    return m_driverController.getY();
  }

  public double getRightVelocity() {
    return m_driveSubsystem.getFrontRightRate();
  }
}
