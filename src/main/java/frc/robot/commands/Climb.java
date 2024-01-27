package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends Command {
    private ClimberSubsystem climberSubsystem;
    private PIDController climbPID = new PIDController(ClimbingConstants.kP, ClimbingConstants.kI,
            ClimbingConstants.kD);
    private double climbTarget;
    private boolean holdPosition;

    public Climb(ClimberSubsystem climberSubsystem, double climbTarget, boolean holdPosition) {
        this.climberSubsystem = climberSubsystem;
        this.climbTarget = climbTarget;
        this.holdPosition = holdPosition;

        climbPID.setTolerance(ClimbingConstants.climbTolerance);
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        double speed = MathUtil.clamp(climbPID.calculate(climberSubsystem.getPosition(), this.climbTarget),
                -ClimbingConstants.climbRateMax, ClimbingConstants.climbRateMax);
        climberSubsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return this.holdPosition ? false : climbPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopMotors();
    }

}
