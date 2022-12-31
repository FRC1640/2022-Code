package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutonSpinUp extends CommandBase{
    private ShooterSubsystem shooterSubsystem;

    public AutonSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setTargetRpm(0); 
        shooterSubsystem.setMotorSpeeds(0.3, 0.4); 
        shooterSubsystem.motionProfileAngleCalculated();
        shooterSubsystem.pointTurret();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setMotorSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
