package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
public class Intake extends CommandBase {
    
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    public Intake (IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // System.out.println("1 " + intakeSubsystem.getProx1());
        // System.out.println("2 " + indexerSubsystem.getProx2());
        intakeSubsystem.runIntakeMotor();
        intakeSubsystem.runRightMotor();
        intakeSubsystem.deployIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
        intakeSubsystem.stopRightMotor();
        intakeSubsystem.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}