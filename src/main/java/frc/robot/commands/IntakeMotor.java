package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeMotorSubsystem;

public class IntakeMotor extends Command{
    public IntakeMotorSubsystem intakeMotorSubsystem;
    public boolean finished = false;

    public IntakeMotor(IntakeMotorSubsystem intakeMotorSubsystem){
        this.intakeMotorSubsystem = intakeMotorSubsystem;
        addRequirements(this.intakeMotorSubsystem);
        
    }

	@Override
	public void execute() {
        intakeMotorSubsystem.setIntakeMotor(IntakeConstants.intakeMotorSpeed);
        if(intakeMotorSubsystem.currentlyAcquired()){
            finished = true;
        }
    }


    @Override
	public boolean isFinished() {
		return finished;
	}

    @Override
	public void end(boolean interrupted) {
        intakeMotorSubsystem.setIntakeMotor(0.0);
	}




}
