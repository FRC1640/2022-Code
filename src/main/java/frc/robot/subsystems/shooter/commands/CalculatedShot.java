package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CalculatedShot extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    long initTime;
    public CalculatedShot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        shooterSubsystem.setupMotorSpeed();
        shooterSubsystem.setupMotorSpeed2();
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.initAverage();
    }

    @Override
    public void execute() {
        //System.out.println(shooterSubsystem.atTargetAngle());
        shooterSubsystem.pointTurret();
        shooterSubsystem.setPipeline();
        shooterSubsystem.setTargetRpm(1700);
        shooterSubsystem.setMotorSpeedsCalculated();
        shooterSubsystem.motionProfileAngleCalculated();
        //shooterSubsystem.setMotorSpeedsCalculated(); //0.37, 0.45
        // System.out.println("Angle " + shooterSubsystem.getAngle());
        // System.out.println(shooterSubsystem.getActualRpm());
        // if (System.currentTimeMillis() > initTime){
        //     initTime = System.currentTimeMillis() + 400;
        //     shooterSubsystem.logShot();
        // }

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
