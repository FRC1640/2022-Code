package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utilities.Logger;

public class ResetGyro extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public ResetGyro (DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(driveSubsystem, shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.resetGyro();
        driveSubsystem.setOffset(0);
        shooterSubsystem.resetGyro();
        Logger.log("Gyro reset!");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
