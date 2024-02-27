package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.flywheel.FlywheelStartIntakeCommand;
import frc.robot.commands.flywheel.FlywheelStopIntakeCommand;
import frc.robot.commands.flywheel.GetIntakeStatusFlywheel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GetFullIntakeRoutine {
    public static Command fullIntakeSequence(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem){
        double extendIntakeAngle = 30.0;
        double restIntakeAngle = 0.0;

        double flyWheelShoulderRecieveAngle = 0.0;
        double flyWheelElbowRecieveAngle = 0.0;

        double flyWheelShoulderShootAngle = 30.0;
        double flyWheelElbowShootAngle = 20.0;
        //idk if these are correct values
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new SetArmAngleCommand(intakeSubsystem, extendIntakeAngle),
                    new StartIntakeCommand(intakeSubsystem)
                ),
                
                new InstantCommand(
                    () -> {flywheelSubsystem.setArmSetpoint(flyWheelShoulderRecieveAngle, flyWheelElbowRecieveAngle);}                   
                )

            ),

            new GetIntakeStatus(intakeSubsystem),
            new ParallelCommandGroup(
                new FlywheelStartIntakeCommand(flywheelSubsystem),
                new StopIntakeCommand(intakeSubsystem)
            ),
            new SequentialCommandGroup(
                new GetIntakeStatusFlywheel(flywheelSubsystem),
                new FlywheelStopIntakeCommand(flywheelSubsystem)
            ),
            new InstantCommand(
                    () -> {flywheelSubsystem.setArmSetpoint(flyWheelShoulderShootAngle, flyWheelElbowShootAngle);}                   
            )

        );
        
    }
}
