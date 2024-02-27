package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FlywheelSubsystem;

public class Autos{
    public static Command getShootCommand(FlywheelSubsystem flywheelSubsystem){
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    flywheelSubsystem.startShooter();

                }
            ),
            new WaitCommand(2.0),
            new InstantCommand(
                () -> {
                    flywheelSubsystem.stopShooter();
                }
            )

        );
    }
}
