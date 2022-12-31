package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IndexShooter extends CommandBase {

    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    XboxController controller= new XboxController(1);
    ShooterSubsystem shooterSubsystem;
    int id;
    public IndexShooter (IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //boolean check = false;
        // if (!indexerSubsystem.getProx2()){
        //     check = true;
        // }
        // if (check && indexerSubsystem.getProx2()){
        //     check = false;
            
        //     id += 1;
        // }
        // System.out.println("ID: " + id);
        if(controller.getLeftBumper()) {
            indexerSubsystem.runUppererMotorShootFast();
            intakeSubsystem.runRightMotorShootFast();
        } else {
            indexerSubsystem.runUppererMotorShoot();
            intakeSubsystem.runRightMotorShoot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopRightMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}