package frc.robot.subsystems.drive.commands;

import javax.sound.midi.SysexMessage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveDuration extends CommandBase{
    DriveSubsystem driveSubsystem;
    double x;
    double y;
    double rot;
    double duration;
    double time;
    public DriveDuration(DriveSubsystem driveSubsystem, double x, double y, double rot, double duration) {
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.duration = duration;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        time = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        driveSubsystem.drive(x, y, rot, false);
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }
    @Override
    public boolean isFinished(){ 
        return System.currentTimeMillis() - time >= duration;
    }
}