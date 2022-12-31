package frc.robot.autonomous.paths;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.auton.AutonClimber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveToBall;
import frc.robot.subsystems.drive.commands.DriveToOneBall;
import frc.robot.subsystems.drive.commands.PointWheels;
import frc.robot.subsystems.drive.commands.ResetOdometry;
import frc.robot.subsystems.drive.commands.SetOffset;
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.AutonBallDump;
import frc.robot.subsystems.shooter.commands.OneShotAlign;
import frc.robot.subsystems.shooter.commands.PointTurret;
public class DefensiveFourBall implements PathInterface{
    PathPlannerTrajectory dFourBall1 = PathPlanner.loadPath("D4Ball1", 6, 4);
    PathPlannerTrajectory dFourBall2 = PathPlanner.loadPath("D4Ball2", 6, 4);
    PathPlannerTrajectory dFourBall3 = PathPlanner.loadPath("D4Ball3", 6, 4);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; // 10.875"
    public static final double y = 0.301625; // 11.875"
    private final DriveSubsystem swerve = RobotContainer.drive;
    private final IndexerSubsystem indexerSubsystem = RobotContainer.indexer;
    private final IntakeSubsystem intakeSubsystem = RobotContainer.intake;
    private final ShooterSubsystem shooterSubsystem = RobotContainer.shooter;
    private final ClimberSubsystem climberSubsystem = RobotContainer.climber;
    PathPlannerState dFourBall1State = new PathPlannerState();
    PathPlannerState dFourBall2State = new PathPlannerState();
    PathPlannerState dFourBall3State = new PathPlannerState();
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
    @Override
    public Command getAutoCommand(){

        dFourBall1State = dFourBall1.getInitialState();
        Pose2d dFourBall1Pose = new Pose2d(dFourBall1State.poseMeters.getTranslation(), dFourBall1State.holonomicRotation);

        dFourBall2State = dFourBall2.getInitialState();
        Pose2d dFourBall2Pose = new Pose2d(dFourBall2State.poseMeters.getTranslation(), dFourBall2State.holonomicRotation);

        dFourBall3State = dFourBall3.getInitialState();
        Pose2d dFourBall3Pose = new Pose2d(dFourBall3State.poseMeters.getTranslation(), dFourBall3State.holonomicRotation);

        var thetaController = new ProfiledPIDController(10, 0.0, 0.0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand dFourBall1Command = new SwerveControllerCommand(dFourBall1,
                swerve::getPose, // Functional interface to feed supplier
                kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController,
                swerve::setModuleStates, swerve);

        SwerveControllerCommand dFourBall2Command = new SwerveControllerCommand(dFourBall2,
                swerve::getPose, // Functional interface to feed supplier
                kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController,
                swerve::setModuleStates, swerve);

        SwerveControllerCommand dFourBall3Command = new SwerveControllerCommand(dFourBall3,
                swerve::getPose, // Functional interface to feed supplier
                kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController,
                swerve::setModuleStates, swerve);

        Command setOffset = new SetOffset(swerve, shooterSubsystem, 224.5); 

        Command climb = new AutonClimber(climberSubsystem, -25);

        Command resetOdoD4Ball1 = new ResetOdometry(swerve, dFourBall1Pose);
        Command resetOdoD4Ball2 = new ResetOdometry(swerve, dFourBall2Pose);
        Command resetOdoD4Ball3 = new ResetOdometry(swerve, dFourBall3Pose);

        Command intake1 = new DriveToBall(intakeSubsystem, indexerSubsystem, swerve, 0.3, 5000);
        Command intake2 = new DriveToOneBall(intakeSubsystem, indexerSubsystem, swerve, 0.3, 3000);
        Command intake3 = new DriveToBall(intakeSubsystem, indexerSubsystem, swerve, 0.3, 3000);

        Command pointTurret1 = new PointTurret(shooterSubsystem, 0);

        Command align1 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        Command align2 = new AutonBallDump(shooterSubsystem, intakeSubsystem, indexerSubsystem);
    
        Command stop1 = new Stop(swerve);

        ParallelDeadlineGroup intakeGroup1 = new ParallelDeadlineGroup(intake1, pointTurret1, intake1, climb);

        SequentialCommandGroup defensiveFourBall = new SequentialCommandGroup(
                                setOffset, intakeGroup1, align1,
                                resetOdoD4Ball1, dFourBall1Command, intake2,
                                resetOdoD4Ball2, dFourBall2Command, intake3,
                                resetOdoD4Ball3, dFourBall3Command, stop1, align2);

        return defensiveFourBall;
    }
}
