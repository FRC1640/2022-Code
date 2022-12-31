package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutonBallDump extends CommandBase{

    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    public IntakeSubsystem intakeSubsystem;

    public AutonBallDump(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(shooterSubsystem, indexerSubsystem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setTargetRpm(500); //500
        shooterSubsystem.setMotorSpeeds(0.2, 0.2); //0.2, 0.2
        shooterSubsystem.motionProfileAngle(-55); //-55
        shooterSubsystem.pointTurretDeadReckoning(0);
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
