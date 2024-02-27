package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.flywheel.FeederStartCommand;
import frc.robot.commands.flywheel.FlywheelShootCommand;
import frc.robot.subsystems.FlywheelSubsystem;

public class FullFlywheelShoot {
    SequentialCommandGroup fullShoot = new SequentialCommandGroup ();
    private FlywheelSubsystem flywheel;
        
    public FullFlywheelShoot () {
        flywheel = new FlywheelSubsystem ();
        this.fullShoot.addCommands (new FlywheelShootCommand (flywheel));
        this.fullShoot.addCommands (new FeederStartCommand(flywheel));
    }
    
    public Command command () {
        return this.fullShoot;
    }
}