package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utilities.Logger;

public class SetOffset extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private double offset;

    public SetOffset(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, double offset) {
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.offset = offset;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.setOffset(offset);
        shooterSubsystem.setOffset(offset);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
