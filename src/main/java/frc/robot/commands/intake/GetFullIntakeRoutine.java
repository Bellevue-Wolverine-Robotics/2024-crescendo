package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GetFullIntakeRoutine {







    public static Command fullIntakeSequence(IntakeSubsystem intakeSubsystem, FlywheelSubsystem flywheelSubsystem){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmAngleCommand(intakeSubsystem, 30),
                new

            )
            
        )
        
    }
}
