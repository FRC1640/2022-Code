package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PointTurretAndAngle extends CommandBase{

    private ShooterSubsystem shooterSubsystem;
    private double position;
    private double angle;

    public PointTurretAndAngle(ShooterSubsystem shooterSubsystem, double position, double angle) {
        this.shooterSubsystem = shooterSubsystem;
        this.position = position;
        this.angle = angle;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.pointTurretDeadReckoning(position);
        shooterSubsystem.motionProfile(angle);
        shooterSubsystem.setTargetRpm(0);
        shooterSubsystem.setMotorSpeeds(0.3, 0.4);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.pointedAtTarget() && shooterSubsystem.atTargetAngle();
    }
}
