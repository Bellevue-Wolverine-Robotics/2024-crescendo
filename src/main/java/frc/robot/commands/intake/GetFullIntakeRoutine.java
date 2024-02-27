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
                    /*
                     * AFTER INTAKE IS IN RECIEVING POSITION, 
                     * INTAKE WHEELS STARTS SPINNING
                     */
                ),
                
                new InstantCommand(
                    () -> {flywheelSubsystem.setArmSetpoint(flyWheelShoulderRecieveAngle, flyWheelElbowRecieveAngle);}    
                    /*
                    *GET FLYWHEEEL IN POSITION TO RECIEVE
                    */               
                )

            ),
            /*
             * AT THIS POINT INTAKE IS IN RECIEVING POSITION AND THE INTAKE IS SPINNNING
             * THE FLYWHEEL IS IN POSITION TO RECIEVE FROM INTAKE
             */
            new GetIntakeStatus(intakeSubsystem),
            /*
             * AT THIS POINT ARM HAS ACQUIRED NOTE(VERIFIED BY LIMIT SWITCH)
             */
            new SetArmAngleCommand(intakeSubsystem, restIntakeAngle),
            /*
             * AT THIS POINT THE INTAKE ARM IS AT REST POSITION AND IN POSITION FOR FLYWHEEL INTAKE TO RECIEVE
             */
            new ParallelCommandGroup(
                new FlywheelStartIntakeCommand(flywheelSubsystem),
                new StopIntakeCommand(intakeSubsystem)
            ),
            /*
             * INTAKE MOTOR WILL STOP BECAUSE IT HAS ACUIRED NOTE 
             * AT THIS POINT THE FLYWHEEL WILL START ACQURING THE NOTE FROM THE INTAKE
             */
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
