// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DebugClose;
import frc.robot.commands.FullRoutines;
import frc.robot.commands.climber.ClimberExtendCommand;
import frc.robot.commands.climber.ClimberRetractCommand;
import frc.robot.commands.drivetrain.ArcadeDriveCommand;
import frc.robot.commands.flywheel.FlywheelAimIntakeReceiveCommand;
import frc.robot.commands.flywheel.FlywheelAimSpeakerCommand;
import frc.robot.commands.flywheel.FlywheelClimbModeCommand;
import frc.robot.commands.flywheel.FlywheelMoveToMakeSpaceForIntakeCommand;
import frc.robot.commands.flywheel.FlywheelShoot;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.StowIntakeCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SmartOperatorButtons;
import frc.robot.constants.IOConstants.DriverButtonConstants;
import frc.robot.constants.IOConstants.JoystickPortConstants;
import frc.robot.constants.IOConstants.OperatorButtonConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final SendableChooser<Command> m_autoChooser;

  private final Debug debugLogger = new Debug("DebugDriveSubsystem.txt");

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(debugLogger);
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final FlywheelSubsystem m_flyWheelSubsystem = new FlywheelSubsystem();

  private final CommandJoystick m_driverController = new CommandJoystick(JoystickPortConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(
      JoystickPortConstants.kOperatorControllerPort);



  private final String smartDashboardDebugKey = "Debug Mode";
  private final String smartDashboardDebugKeyLabel = "Debug Mode Label";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;


  public RobotContainer() {
    configureBindings();
    smartDashBoardBinding();

    NamedCommands.registerCommand("Intake", FullRoutines.getFullIntakeRoutine(m_intakeSubsystem, m_flyWheelSubsystem));
    NamedCommands.registerCommand("ShootSpeaker", FullRoutines.getShootAtSpeakerRoutine(m_flyWheelSubsystem));
    NamedCommands.registerCommand("TestPrint",
        new InstantCommand(() -> System.out.println("TEST")));

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);




  }



  private void configureBindings() {
    // Driving
    m_driveSubsystem.setDefaultCommand(
        new ArcadeDriveCommand(m_driveSubsystem, () -> getArcadeDriveSpeeds().getFirst(),
            () -> getArcadeDriveSpeeds().getSecond()));

    // Climber
    m_operatorController.button(OperatorButtonConstants.kClimbUpButton)
        .whileTrue(new ClimberExtendCommand(m_climberSubsystem));
    m_operatorController.button(OperatorButtonConstants.kClimbDownButton)
        .whileTrue(new ClimberRetractCommand(m_climberSubsystem));

    // Intake
    m_operatorController.button(OperatorButtonConstants.kFullIntakeCycle)
        .onTrue(FullRoutines.getFullIntakeRoutine(m_intakeSubsystem, m_flyWheelSubsystem));

    m_operatorController.button(OperatorButtonConstants.kFullShootCycle)
        .onTrue(FullRoutines.getShootAtSpeakerRoutine(m_flyWheelSubsystem));

    m_flyWheelSubsystem.setDefaultCommand(
        new InstantCommand(() -> m_flyWheelSubsystem.debugShoulder(m_operatorController), m_flyWheelSubsystem));



    m_operatorController.button(9).onTrue(new FlywheelAimSpeakerCommand(m_flyWheelSubsystem));
    
    //m_operatorController.button(6).onTrue(new InstantCommand(m_flyWheelSubsystem::startShooter, m_flyWheelSubsystem));
    //m_operatorController.button(7).onTrue(new InstantCommand(m_flyWheelSubsystem::stopShooter, m_flyWheelSubsystem));
    m_operatorController.button(6).onTrue(new FlywheelAimIntakeReceiveCommand(m_flyWheelSubsystem));
    m_operatorController.button(7).onTrue(new StowIntakeCommand(m_intakeSubsystem));



    m_operatorController.button(10).onTrue(FullRoutines.prepareToClimb(m_intakeSubsystem));
    m_operatorController.button(11).onTrue(new FlywheelClimbModeCommand(m_flyWheelSubsystem));



    m_operatorController.button(8).onTrue(new FlywheelAimIntakeReceiveCommand(m_flyWheelSubsystem));




    m_operatorController.button(1).whileTrue(
        //new InstantCommand(() -> {m_flyWheelSubsystem.aimLowerPosition();})
        new SequentialCommandGroup(


          new InstantCommand(() -> {m_flyWheelSubsystem.aimArmToIntake();}),          
          new InstantCommand(() -> {m_flyWheelSubsystem.startShooter();}),
          new WaitCommand(0.3),
          new InstantCommand(() -> {m_flyWheelSubsystem.aimLowerPosition();}),          
          new WaitCommand(0.05),

          new InstantCommand(() -> {m_intakeSubsystem.startFeedIntoFlywheel();}),
          new InstantCommand(() -> {m_flyWheelSubsystem.startFeeder();})
          //new InstantCommand(() -> {m_flyWheelSubsystem.aimLowerPosition();})

          //new WaitCommand(2.0),


        )
    );

    m_operatorController.button(1).whileFalse(
      new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_flyWheelSubsystem.stopShooter();
          m_flyWheelSubsystem.stopFeeder();
          m_intakeSubsystem.stopIntakeMotor();
        }),
        new InstantCommand(() -> {m_flyWheelSubsystem.aimArmToIntake();})
      )
    );




  }

  public void smartDashBoardBinding() {
    SmartDashboard.putData("Save logging info", new DebugClose(debugLogger));

    SmartDashboard.putString(smartDashboardDebugKeyLabel, DebugSettings.debugMode? "DEBUG MODE": "RELEASE MODE");
    SmartDashboard.putData(smartDashboardDebugKey, new InstantCommand(() -> {
        DebugSettings.debugMode = !DebugSettings.debugMode;
        SmartDashboard.putString(smartDashboardDebugKeyLabel, DebugSettings.debugMode? "DEBUG MODE": "RELEASE MODE");

    }));
    smartDashBoardOperatorBindings();
  }

  public void smartDashBoardOperatorBindings(){
    SmartDashboard.putData(SmartOperatorButtons.fullIntakeRoutine, FullRoutines.getFullIntakeRoutine(m_intakeSubsystem, m_flyWheelSubsystem));


    SmartDashboard.putData(SmartOperatorButtons.flywheelStart, new InstantCommand(m_flyWheelSubsystem::startShooter, m_flyWheelSubsystem));
    SmartDashboard.putData(SmartOperatorButtons.flywheelStop, new InstantCommand(m_flyWheelSubsystem::stopShooter, m_flyWheelSubsystem));

    SmartDashboard.putData(SmartOperatorButtons.FlywheelAimSpeaker, new FlywheelAimSpeakerCommand(m_flyWheelSubsystem));


    SmartDashboard.putData(SmartOperatorButtons.FlywheelMoveToMakeSpaceForIntakeCommand, new FlywheelMoveToMakeSpaceForIntakeCommand(m_flyWheelSubsystem));
    SmartDashboard.putData(SmartOperatorButtons.prepareToClimb, FullRoutines.prepareToClimb(m_intakeSubsystem));

    SmartDashboard.putData(SmartOperatorButtons.extendIntake, new ClimberExtendCommand(m_climberSubsystem));
    SmartDashboard.putData(SmartOperatorButtons.retractIntake, new ClimberRetractCommand(m_climberSubsystem));
  }





  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /**
   * Returns arcade drive speeds based on throttles and squaring selected
   * 
   * @return A pair of the x speed and the z rotation
   */
  private Pair<Double, Double> getArcadeDriveSpeeds() {
    double xSpeed = -m_driverController.getY();
    double zRotation = -m_driverController.getX();

    /*
    if !(DriverStation.getStickButton(what button is this?) {
      double rotationIntensityMult = 1 //Higher value means faster decrease in rotation as speed increases
      zRotation = 1 / ((1 / zRotation) * (rotationIntensityyMult * sqrt(xSpeed) + 1)) //Decreases rotation based on speed increase (more speed = less rotation) for better handling 
    }
    */
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
}
