package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class LowFender extends CommandBase{
    private ShooterSubsystem shooterSubsystem;

    public LowFender(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setTargetRpm(500); //500
        shooterSubsystem.setMotorSpeeds(0.2, 0.2); //0.2, 0.2
        shooterSubsystem.motionProfileAngle(-55); //-55
        shooterSubsystem.pointTurretDeadReckoning(0);
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
