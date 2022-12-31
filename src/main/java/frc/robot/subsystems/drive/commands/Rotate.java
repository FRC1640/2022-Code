package frc.robot.subsystems.drive.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.SPI;
public class Rotate extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private double rotation;
    PIDController pidController = new PIDController(0.0015, 0, 0.0);
    public Rotate (DriveSubsystem driveSubsystem, double rotation) {
        this.driveSubsystem = driveSubsystem;
        this.rotation = rotation;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = pidController.calculate(driveSubsystem.getGyroAngleDegrees(), rotation);
        // System.out.println(rotation);
        driveSubsystem.drive(0, 0, speed, false);
        // System.out.println("Gyro: " + driveSubsystem.getGyroAngleDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
        // System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveSubsystem.getGyroAngleDegrees() - rotation) <= 5;
    }
}
