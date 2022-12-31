package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ToggleSolenoid extends CommandBase{
    IntakeSubsystem intakeSubsystem;

    public ToggleSolenoid (IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //System.out.println(intakeSubsystem.getSolenoid());
        if (intakeSubsystem.getSolenoid()){
            intakeSubsystem.retractIntake();
        }
        else{
            intakeSubsystem.deployIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
