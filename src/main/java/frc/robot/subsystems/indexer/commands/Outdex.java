package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.Intake;

public class Outdex extends CommandBase {

    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    
    public Outdex (IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexerSubsystem.reverseIndexer();
        intakeSubsystem.reverseIndexer();
        intakeSubsystem.deployIntake();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopIntakeMotor();
        intakeSubsystem.stopRightMotor();
        intakeSubsystem.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}