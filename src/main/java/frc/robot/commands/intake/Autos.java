package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Autos {
    public static Command getShootCommand(IntakeSubsystem intakeSubsystem){
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    intakeSubsystem.setIntakeMotorSpeed(-1.0);

                }
            ),
            new WaitCommand(2.0),
            new InstantCommand(
                () -> {
                    intakeSubsystem.setIntakeMotorSpeed(1.0);
                }
            )

        );
    }
}
