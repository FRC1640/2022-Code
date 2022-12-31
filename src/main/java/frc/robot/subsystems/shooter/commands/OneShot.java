package frc.robot.subsystems.shooter.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class OneShot extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private long initTime;
    boolean val = false;
    boolean initProx;
    public IntakeSubsystem intakeSubsystem;
    public OneShot(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(shooterSubsystem, indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("init");
        initProx = indexerSubsystem.getProx2();
    }

    @Override
    public void execute() {
        System.out.println("execute");
        shooterSubsystem.motionProfileAngle(-40);
        // shooterSubsystem.pointTurret();
        shooterSubsystem.setTargetRpm(2000);
        shooterSubsystem.setMotorSpeeds(0.43, 0.43);
        // shooterSubsystem.setPipeline();
        if (shooterSubsystem.atTargetAngle() && shooterSubsystem.atTargetRpm()){
            indexerSubsystem.runUppererMotor();
            intakeSubsystem.runRightMotor();
        }        
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopRightMotor();
        shooterSubsystem.setMotorSpeeds(0, 0);
        System.out.println("interrupted " + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (indexerSubsystem.getProx2() != initProx && val == false){
            val = true;
            initTime = System.currentTimeMillis();
            System.out.println("change");
        }
        // System.out.println("diff " + (System.currentTimeMillis() - initTime));
        return val && System.currentTimeMillis() - initTime >= 1500;
    }
}
