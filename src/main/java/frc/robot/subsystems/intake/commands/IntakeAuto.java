package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeAuto extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    DriveSubsystem driveSubsystem;
    long initTime;
    long duration;

    public IntakeAuto(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, DriveSubsystem driveSubsystem, long duration) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.duration = duration;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntakeMotor();
        intakeSubsystem.runRightMotor();
        intakeSubsystem.deployIntake();

        if(!(!indexerSubsystem.getProx2() == true)) {
            indexerSubsystem.runUppererMotor();
        } else {
            indexerSubsystem.stopIndexer();
        }

        driveSubsystem.drive(-0.11, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
        intakeSubsystem.stopRightMotor();
        intakeSubsystem.retractIntake();
        driveSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return (!intakeSubsystem.getProx1() == true && !indexerSubsystem.getProx2()) || System.currentTimeMillis() - initTime >= duration;
    }
    
}
