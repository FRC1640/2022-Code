package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class SetIndexerSpeed extends CommandBase {

    IndexerSubsystem indexerSubsystem;
    boolean isFast;

    public SetIndexerSpeed(IndexerSubsystem indexerSubsystem, boolean isFast) {
        this.indexerSubsystem = indexerSubsystem;
        this.isFast = isFast;
        addRequirements(indexerSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexerSubsystem.setFast(isFast);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
