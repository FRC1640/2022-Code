package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class LowerClimber extends CommandBase{
    private ClimberSubsystem climberSubsystem;
    private double initEncoder = 0;

    public LowerClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        initEncoder = climberSubsystem.getPosition();
        climberSubsystem.engageBrake();
    }

    @Override
    public void execute() {
        
        if (climberSubsystem.getPosition() <= -22){
            climberSubsystem.setMotors(1.0);
        }
        else{
            climberSubsystem.setMotors(0.75);
        }

        if (climberSubsystem.getPosition() >= -80) {
            climberSubsystem.disengageBrake();
            System.out.println("AHH");
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
        climberSubsystem.quickSetup();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.limit();
    }
}
