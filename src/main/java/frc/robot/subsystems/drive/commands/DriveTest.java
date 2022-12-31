package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveTest extends CommandBase {

    DriveSubsystem driveSubsystem;

    public DriveTest(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        long time = System.currentTimeMillis() * 1000;
        double angle = 0;

        driveSubsystem.setSpeeds(1.0);
        driveSubsystem.pointWheels(angle);

        if(time % 30 == 0) {
            angle += 90;
            if (angle > 360) {
                angle = 0;
            }
        }
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}