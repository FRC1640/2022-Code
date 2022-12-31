package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ReverseClimber extends CommandBase{
    private ClimberSubsystem climberSubsystem;
    private double initEncoder = 0;

    public ReverseClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        initEncoder = climberSubsystem.getPosition();
    }

    @Override
    public void execute() {
        System.out.println("init " + initEncoder);
        if (climberSubsystem.getPosition() >= -10){
            climberSubsystem.setMotors(-0.5);
        }

        if(climberSubsystem.getPosition() <= initEncoder - 3) {
            climberSubsystem.engageBrake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
        climberSubsystem.quickSetup();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.getPosition() <= -10;
    }
}
