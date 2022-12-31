package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ClimberRotateTurret extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private ClimberSubsystem climberSubsystem;
    boolean check = true;
    boolean done = false;
    long initTime;
    public ClimberRotateTurret(ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //System.out.println("turret");
        shooterSubsystem.pointTurretDeadReckoning(10);
        shooterSubsystem.motionProfileAngle(0);
        System.out.println(check);
        if (shooterSubsystem.getAngle() >= -9 && Math.abs(shooterSubsystem.getTurretPosition() - 25) <= 6 && check){
            initTime = System.currentTimeMillis();
            climberSubsystem.setVertical(false);
            check = false;
        }
        if (System.currentTimeMillis() - initTime >= 500 && !check){
            climberSubsystem.setVertical(true);
        }
        if (System.currentTimeMillis() - initTime >= 1000 && !check){
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        check = true;
        done = false;
        initTime = 0;
        shooterSubsystem.pointTurretController(0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
