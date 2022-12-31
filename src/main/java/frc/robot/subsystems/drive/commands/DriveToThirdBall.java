package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Jetson;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DriveToThirdBall extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private long initTime = 0;
    private long duration = 0;

    private double speed;
    private long time;
    private PIDController horizontalPid = new PIDController(0.15, 0.0, 0.0);
    private PIDController distancePid = new PIDController(0.00065, 0.0, 0.0);

    private double x;

    public DriveToThirdBall(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, double speed, long duration) {
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        this.duration = duration;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        double x = driveSubsystem.getPixyVoltage();
        double y = 0;

        //System.out.println(driveSubsystem.getPixyVoltage());

        double xSpeed = horizontalPid.calculate(x, 1.5);

        // System.out.println(xSpeed);

        intakeSubsystem.runIntakeMotor();
        intakeSubsystem.runRightMotor();
        intakeSubsystem.deployIntake();

        if(driveSubsystem.getPixyVoltage() <= 0.1) {
            driveSubsystem.drive(0, 0, 0, false);
            driveSubsystem.xWheels();
        } else {
            driveSubsystem.drive(speed, xSpeed, 0, false);
        }

        // if(Jetson.get() != null) {
        //     x = Jetson.get().getX() - 640;
        //     y= Jetson.get().getY();

        //     double xSpeed = horizontalPid.calculate(x, 0);
        //     double ySpeed = distancePid.calculate(y, 720);
        //     driveSubsystem.drive(-ySpeed, -xSpeed, 0.0, false);
        // } else {
        //     driveSubsystem.drive(0.0, 0.0, 0.0, true);
        // }

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
