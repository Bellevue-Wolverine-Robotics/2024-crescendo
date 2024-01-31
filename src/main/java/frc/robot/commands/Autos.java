// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.commands.IntakeArm;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


  public static Command IntakeSequence(IntakeArmSubsystem m_intakeArmSubsystem, IntakeMotorSubsystem intakeMotorSubsystem){
      return new SequentialCommandGroup(new IntakeArm(m_intakeArmSubsystem, IntakeConstants.intakePositionDeployed), 
                                        new IntakeMotor(intakeMotorSubsystem), 
                                        new IntakeArm(m_intakeArmSubsystem, IntakeConstants.intakePositionStowed));
  }
}
   
