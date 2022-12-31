package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class RetractClimber extends CommandBase {

    private ClimberSubsystem climberSubsystem;
    private double distance;

    public RetractClimber(ClimberSubsystem climberSubsystem, double distance) {
        this.climberSubsystem = climberSubsystem;
        this.distance = distance;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climberSubsystem.motionProfile(distance);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.getPosition() <= distance + 5;
    }
    
}
