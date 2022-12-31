package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class MultiShot extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private long initTime;
    boolean val = true;
    public IntakeSubsystem intakeSubsystem;
    public MultiShot(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(shooterSubsystem, indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooterSubsystem.motionProfileAngle(0);
        shooterSubsystem.pointTurret();
        shooterSubsystem.setTargetRpm(2000);
        shooterSubsystem.setMotorSpeeds(0.43, 0.43);
        shooterSubsystem.setPipeline();
        if (shooterSubsystem.atTargetAngle() && shooterSubsystem.atTargetRpm() && shooterSubsystem.pointedAtTarget()){
            indexerSubsystem.runUppererMotor();
            intakeSubsystem.runRightMotor();
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopRightMotor();
        shooterSubsystem.setMotorSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        if (indexerSubsystem.getProx2() && val == false){
            val = true;
            initTime = System.currentTimeMillis();

        }
        if (indexerSubsystem.getProx2() == false){
            val = false;
            initTime = 0;
        }
        return initTime - System.currentTimeMillis() >= 2500;
    }
}
