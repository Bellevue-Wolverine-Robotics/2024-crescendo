package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullFlywheelShoot {
    SequentialCommandGroup fullShoot = new SequentialCommandGroup ();
        
    public FullFlywheelShoot () {
        fullShoot.addCommands (new FlywheelShootCommand ());
        fullShoot.addCommand(new FeederStartCommand ());
    }
    
    public Command command () {
        return fullShoot;
    }
}