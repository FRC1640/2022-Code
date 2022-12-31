package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IndexShooterFast extends CommandBase {

    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    XboxController controller= new XboxController(1);
    
    public IndexShooterFast(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // System.out.print(controller.getLeftBumper());
        // if(controller.getLeftBumper()) {
        //     indexerSubsystem.runUppererMotorShootFast();
        //     intakeSubsystem.runRightMotorShootFast();
        // } else {
        //     indexerSubsystem.runUppererMotorShoot();
        //     intakeSubsystem.runRightMotorShootFast();
        // }
        
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