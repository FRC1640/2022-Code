package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Jetson;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DriveToOneBall extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    long time;
    private double speed;
    private long initTime = 0;
    private long duration = 0;
    
    private PIDController horizontalPid = new PIDController(0.15, 0.0, 0.0);
    private PIDController distancePid = new PIDController(0.00065, 0.0, 0.0);

    private double x;

    public DriveToOneBall(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, DriveSubsystem driveSubsystem, double speed, long duration) {
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
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

        double xSpeed = horizontalPid.calculate(x, 1.5);

        intakeSubsystem.runIntakeMotor();
        intakeSubsystem.runRightMotor();
        intakeSubsystem.deployIntake();

        // if(driveSubsystem.getPixyVoltage() <= 0.1) {
        //     driveSubsystem.drive(0, 0, 0, false);
        //     driveSubsystem.xWheels();
        // } else {
        // }

        
        driveSubsystem.drive(speed, xSpeed, 0, false);

        if(!(!indexerSubsystem.getProx2() == true)) {
            indexerSubsystem.runUppererMotor();
        } else {
            indexerSubsystem.stopIndexer();
        }

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
        intakeSubsystem.stopIntakeMotor();
        intakeSubsystem.stopRightMotor();
        indexerSubsystem.stopIndexer();
        intakeSubsystem.retractIntake();
        driveSubsystem.xWheels();
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getProx1() == true || System.currentTimeMillis() - initTime >= duration;
    }
    
}
