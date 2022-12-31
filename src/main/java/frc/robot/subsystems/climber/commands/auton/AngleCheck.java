package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class AngleCheck extends CommandBase{
    private ClimberSubsystem climberSubsystem;
    private double greater;
    private double less;
    public AngleCheck(ClimberSubsystem climberSubsystem, double greater, double less) {
        this.climberSubsystem = climberSubsystem;
        this.greater = greater;
        this.less = less;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.getGyroRoll() >= greater && climberSubsystem.getGyroRoll() <= less;
    }
}
