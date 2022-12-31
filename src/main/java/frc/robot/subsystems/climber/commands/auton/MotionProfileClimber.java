package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class MotionProfileClimber extends CommandBase{
    private ClimberSubsystem climberSubsystem;
    private double distance;
    private long initTime;
    private double lowAngle;
    private double highAngle;
    public MotionProfileClimber(ClimberSubsystem climberSubsystem, double distance, double lowAngle, double highAngle) {
        this.climberSubsystem = climberSubsystem;
        this.distance = distance;
        this.highAngle = highAngle;
        this.lowAngle = lowAngle;
        
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        System.out.println("Gyro: " + climberSubsystem.getGyroRoll());
        if (climberSubsystem.getGyroRoll() <= highAngle && climberSubsystem.getGyroRoll() >= lowAngle){
                climberSubsystem.setMotors(-1.0);
        }
        else {
            climberSubsystem.setMotors(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        
        return Math.abs(climberSubsystem.getPosition() - distance) <= 7;
    }
}
