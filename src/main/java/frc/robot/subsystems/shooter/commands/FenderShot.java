package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FenderShot extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private long initTime;
    double speed = 0.4;
    public FenderShot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
            // shooterSubsystem.setTargetRpm(1500); //1750 prime, 1500 deux
            // shooterSubsystem.setMotorSpeeds(0.35, 0.42); //0.37, 0.45 deux, 0.37, 0.445 prime
            // shooterSubsystem.motionProfileAngle(-5); //-3
            // shooterSubsystem.pointTurretPosition(80);
            //TODO
        //hooterSubsystem.pointTurretDeadReckoning(80);
        // System.out.println(shooterSubsystem.get());
        // if (System.currentTimeMillis() > initTime){
        //     initTime = System.currentTimeMillis() + 500;
        //     shooterSubsystem.logShot();
        // }
        // shooterSubsystem.setTargetRpm(500); //500
        // shooterSubsystem.setMotorSpeeds(0.2, 0.2); //0.2, 0.2
        // shooterSubsystem.motionProfileAngle(-55); //-55
        // shooterSubsystem.pointTurretDeadReckoning(0);
        shooterSubsystem.setTargetRpm(5200 * speed);
        shooterSubsystem.pointTurretPosition(80);
        shooterSubsystem.motionProfileAngle(-140);
        shooterSubsystem.setMotorSpeeds(speed, speed + 0.08);
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
