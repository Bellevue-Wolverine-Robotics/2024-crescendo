package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class GetIntakeStatus extends Command {
    

    IntakeSubsystem intakeArm;
    public GetIntakeStatus(IntakeSubsystem intakeArmSubsystem) {
        intakeArm = intakeArmSubsystem;
    } 

    public boolean acuiqredNote(){
        return intakeArm.hasNote();

    }

    public boolean isFinished(){
        return intakeArm.hasNote();
    }
 


}
