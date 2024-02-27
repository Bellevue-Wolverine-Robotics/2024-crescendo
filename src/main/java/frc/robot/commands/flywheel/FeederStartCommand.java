package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

public class FeederStartCommand extends Command {
    private FlywheelSubsystem flywheel;
    private boolean finished;

    public FeederStartCommand (FlywheelSubsystem flywheel) {
        this.finished = false;
        this.flywheel = flywheel;
    }

    public void initialize () {
        this.flywheel.startFeederMotor ();
        this.finished = true;
    }

    public boolean isFinished () {
        return this.finished;
    }
}
