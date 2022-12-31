package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class Index extends CommandBase {

    IndexerSubsystem indexerSubsystem;

    public Index (IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexerSubsystem.runUppererMotor();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Indexer stop");
        indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        
        return !indexerSubsystem.getProx2() == true;
    }
    
}
