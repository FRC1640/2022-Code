package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ExtendClimber extends CommandBase {

    private ClimberSubsystem climberSubsystem;
    private double distance;
    private long initTime;

    public ExtendClimber(ClimberSubsystem climberSubsystem, double distance) {
        this.climberSubsystem = climberSubsystem;
        this.distance = distance;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        climberSubsystem.motionProfile(distance);
        // System.out.println(climberSubsystem.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        // System.out.println(climberSubsystem.getPosition());
        return System.currentTimeMillis() - initTime >= 1000;
    }
    
}
