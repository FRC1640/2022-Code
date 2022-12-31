package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class AutonClimber extends CommandBase {

    private ClimberSubsystem climberSubsystem;
    private double distance;
    private long initTime;
    boolean check = true;
    public AutonClimber(ClimberSubsystem climberSubsystem, double distance) {
        this.climberSubsystem = climberSubsystem;
        this.distance = distance;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        check = true;
        initTime = System.currentTimeMillis();
        
    }

    @Override
    public void execute() {
        // climberSubsystem.motionProfile(distance);
        climberSubsystem.setVertical(true);
        // System.out.println(climberSubsystem.getPosition());
        
        if (climberSubsystem.limit() && check){
            check = false;

        }
        if (!check){
            climberSubsystem.motionProfile(0);
        }
        else{
            climberSubsystem.setMotors(0.3);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        // System.out.println(climberSubsystem.getPosition());
        return System.currentTimeMillis() - initTime >= 1500;
    }
    
}
