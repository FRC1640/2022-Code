package frc.robot.subsystems.climber.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class SetupMotors extends CommandBase{
        
    ClimberSubsystem climberSubsystem;

    public SetupMotors(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climberSubsystem.setupMotor();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
