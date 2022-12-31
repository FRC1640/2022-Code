package frc.robot.autonomous.paths;

import javax.naming.PartialResultException;

import com.fasterxml.jackson.databind.introspect.AnnotationCollector.TwoAnnotations;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.auton.AutonClimber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveDuration;
import frc.robot.subsystems.drive.commands.DriveToBall;
import frc.robot.subsystems.drive.commands.DriveToHP;
import frc.robot.subsystems.drive.commands.DriveToOneBall;
import frc.robot.subsystems.drive.commands.PointWheels;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.drive.commands.ResetOdometry;
import frc.robot.subsystems.drive.commands.SetOffset;
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.IntakeTime;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.AutonSpinUp;
import frc.robot.subsystems.shooter.commands.OneShotAlign;
import frc.robot.subsystems.shooter.commands.OneShotAlign3Ball;
import frc.robot.subsystems.shooter.commands.PointTurret;
import frc.robot.subsystems.shooter.commands.PointTurretAndAngle;
public class FiveBallRed implements PathInterface{
    PathPlannerTrajectory twoBall = PathPlanner.loadPath("5 ball new 1", 3.5, 4);

    PathPlannerTrajectory threeBall = PathPlanner.loadPath("5 ball new 2", 3, 3.2);
    PathPlannerTrajectory fiveBall = PathPlanner.loadPath("5 ball new 3", 3, 3.5);
    PathPlannerTrajectory returnPath = PathPlanner.loadPath("5 ball new 4", 3, 3.5);
    //PathPlannerTrajectory threeBall1 = PathPlanner.loadPath("3Ball1", 6, 4);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; 
    public static final double y = 0.301625; 
    private final DriveSubsystem swerve = RobotContainer.drive;
    private final IndexerSubsystem indexerSubsystem = RobotContainer.indexer;
    private final IntakeSubsystem intakeSubsystem = RobotContainer.intake;
    private final ShooterSubsystem shooterSubsystem = RobotContainer.shooter;
    private final ClimberSubsystem climberSubsystem = RobotContainer.climber;
    PathPlannerState twoBallState = new PathPlannerState();
    PathPlannerState threeBallState = new PathPlannerState();
    PathPlannerState fiveBallState = new PathPlannerState();
    PathPlannerState returnPathState = new PathPlannerState();
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

    @Override
    public Command getAutoCommand(){
        twoBallState = twoBall.getInitialState();
        Pose2d twoBallPose = new Pose2d(twoBallState.poseMeters.getTranslation(), twoBallState.holonomicRotation);

        threeBallState = threeBall.getInitialState();
        Pose2d threeBallPose = new Pose2d(threeBallState.poseMeters.getTranslation(), threeBallState.holonomicRotation);

        fiveBallState = fiveBall.getInitialState();
        Pose2d fiveBallPose = new Pose2d(fiveBallState.poseMeters.getTranslation(), fiveBallState.holonomicRotation);

        returnPathState = returnPath.getInitialState();
        Pose2d returnPathPose = new Pose2d(returnPathState.poseMeters.getTranslation(), returnPathState.holonomicRotation);

        var thetaController = new ProfiledPIDController(5, 0.0, 0.0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand twoBallController = new SwerveControllerCommand(twoBall,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), thetaController,
        swerve::setModuleStates, swerve);

        SwerveControllerCommand threeBallController = new SwerveControllerCommand(threeBall,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), thetaController,
        swerve::setModuleStates, swerve);

        SwerveControllerCommand fiveBallController = new SwerveControllerCommand(fiveBall,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), thetaController,
        swerve::setModuleStates, swerve);

        PPSwerveControllerCommand returnPathController = new PPSwerveControllerCommand(returnPath,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), thetaController,
        swerve::setModuleStates, swerve);

        // SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(threeBall1,
        // swerve::getPose, // Functional interface to feed supplier
        // kDriveKinematics, new PIDController(2, 0.0, 0.0), new PIDController(2, 0.0, 0.0), thetaController,
        // swerve::setModuleStates, swerve);
    
        Command setOffset = new SetOffset(swerve, shooterSubsystem, 90);
        Command setOffsetNone = new SetOffset(swerve, shooterSubsystem, 0);
        Command resetOdo2Ball = new ResetOdometry(swerve, twoBallPose);
        Command resetOdo3Ball = new ResetOdometry(swerve, threeBallPose);
        Command resetOdo5Ball = new ResetOdometry(swerve, fiveBallPose);
        Command resetOdoReturnPath = new ResetOdometry(swerve, returnPathPose);

        Command intake2Balls = new DriveToOneBall(intakeSubsystem, indexerSubsystem, swerve, 0.2, 2500);
        Command intake1Ball = new DriveToOneBall(intakeSubsystem, indexerSubsystem, swerve, 0.45, 2500);
        Command intakeatHP = new DriveToHP(intakeSubsystem, indexerSubsystem, swerve, 0.4, 1500);

        Command intakeWhileDriving = new IntakeTime(intakeSubsystem, indexerSubsystem, 3000);
        Command intakeWhileDriving1 = new IntakeTime(intakeSubsystem, indexerSubsystem, 5000);
        Command intakeWhileDriving2 = new IntakeTime(intakeSubsystem, indexerSubsystem, 5000);
        Command intakeWhileDriving3 = new IntakeTime(intakeSubsystem, indexerSubsystem, 10000);
        Command intakeWhileDriving4 = new IntakeTime(intakeSubsystem, indexerSubsystem, 5000);
        Command intakeWhileDriving5 = new IntakeTime(intakeSubsystem, indexerSubsystem, 5000);

        Command climb = new AutonClimber(climberSubsystem, -25);

        Command stop1 = new Stop(swerve);
        Command stop2 = new Stop(swerve);
        // Command stop3 = new PointWheels(swerve, 0); //WHY? THis isn't a stop
        Command stop4 = new Stop(swerve);
        Command stop5 = new Stop(swerve);
        Command spin1 = new AutonSpinUp(shooterSubsystem);
        Command spin2 = new AutonSpinUp(shooterSubsystem);
        Command spin3 = new AutonSpinUp(shooterSubsystem);

        Command alignShoot1 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        //Command alignShoot2 = new OneShotAlign3Ball(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        Command alignShoot3 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        Command alignShoot2 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        ParallelCommandGroup shootIntake = new ParallelCommandGroup(alignShoot2);
        // ParallelDeadlineGroup intakeWallBallGroup = new ParallelDeadlineGroup(intake2Balls);//, pointTurretWall, climb);
        // ParallelDeadlineGroup threeBallDriveGroup = new ParallelDeadlineGroup(threeBallController, threeBallController);
        // ParallelCommandGroup intake3BallGroup = new ParallelCommandGroup(intake1Ball);
        Command pointTurret1 = new PointTurret(shooterSubsystem, 86);
        Command pointTurret2 = new PointTurret(shooterSubsystem, 20);
        Command pointTurret3 = new PointTurret(shooterSubsystem, 100);
        Command driveBack = new DriveDuration(swerve, -0.6, 0, 0, 1500);
        ParallelDeadlineGroup threeBallGroup = new ParallelDeadlineGroup(threeBallController, threeBallController, pointTurret2);
        ParallelDeadlineGroup twoBallGroup = new ParallelDeadlineGroup(intakeWhileDriving, twoBallController, pointTurret1, intakeWhileDriving, climb);
        //ParallelDeadlineGroup fiveBallGroup = new ParallelDeadlineGroup(fiveBallController, fiveBallController);
        ParallelDeadlineGroup returnPathGroup = new ParallelDeadlineGroup(driveBack, driveBack, pointTurret3);
        SequentialCommandGroup intakeThing = new SequentialCommandGroup(returnPathGroup, stop5, shootIntake);
        ParallelCommandGroup intaking = new ParallelCommandGroup(intakeThing, intakeWhileDriving3);
        //SequentialCommandGroup intakeHPGroup = new SequentialCommandGroup(intakeatHP, stop4); //do we still need to intake backwards

        ParallelDeadlineGroup spinGroup1 = new ParallelDeadlineGroup(stop1, stop1, spin1);
        ParallelDeadlineGroup spinGroup2 = new ParallelDeadlineGroup(stop2, stop2, spin2);
        Command resetGyro = new ResetGyro(swerve, shooterSubsystem);
        ParallelDeadlineGroup spinGroup3 = new ParallelDeadlineGroup(stop4, stop4, spin3);

        
        //ParallelDeadlineGroup spinReturnGroup = new ParallelDeadlineGroup(returnPathGroup, returnPathGroup, intakeWhileDriving1);
        SequentialCommandGroup fiveBall = new SequentialCommandGroup(resetGyro, resetOdo2Ball, setOffset, twoBallGroup, spinGroup3, alignShoot1, 
        threeBallGroup, intake1Ball, spinGroup1, alignShoot3, fiveBallController, intake2Balls, intaking);
            // SequentialCommandGroup fiveBall = new SequentialCommandGroup(
            //                     setOffset, resetOdo2Ball, twoBallGroup, spinGroup1, alignShoot1, //Wall ball and shoot 
            //                     threeBallGroup, spinGroup2, alignShoot2, //Three ball and shoot
            //                     resetOdo5Ball, fiveBallGroup, intakeHPGroup, //Five ball and intake
            //                     resetOdoReturnPath, spinReturnGroup, spinGroup3, alignShoot3); //Return and shoot
        //SequentialCommandGroup fiveBall = new SequentialCommandGroup(resetGyro, resetOdoReturnPath, returnPathController);
        return fiveBall;
    }
}
