package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetIntakeSpeed extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    boolean isFast;

    public SetIntakeSpeed(IntakeSubsystem intakeSubsystem, boolean isFast) {
        this.intakeSubsystem = intakeSubsystem;
        this.isFast = isFast;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSubsystem.setFast(isFast);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
