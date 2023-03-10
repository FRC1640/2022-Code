package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PointWheels extends CommandBase {

    DriveSubsystem driveSubsystem;
    double angle;
    long startTime;
    
    public PointWheels(DriveSubsystem driveSubsystem, double angle) {
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // System.out.println("Point wheels");
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        driveSubsystem.pointWheels(angle);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
        //System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= startTime + 250;
    }



}
