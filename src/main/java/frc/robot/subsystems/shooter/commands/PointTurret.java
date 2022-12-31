package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PointTurret extends CommandBase{

    private ShooterSubsystem shooterSubsystem;
    private double position;

    public PointTurret(ShooterSubsystem shooterSubsystem, double position) {
        this.shooterSubsystem = shooterSubsystem;
        this.position = position;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.pointTurretPosition(position);
        shooterSubsystem.setTargetRpm(0);
        shooterSubsystem.setMotorSpeeds(0.3, 0.4);
        shooterSubsystem.motionProfileAngleCalculated();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.pointedAtTarget();
    }
}
