package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import frc.robot.subsystems.drive.commands.ResetOdometry;
import frc.robot.subsystems.drive.commands.SetOffset;
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.AutonSpinUp;
import frc.robot.subsystems.shooter.commands.OneShotAlign;
import frc.robot.subsystems.shooter.commands.PointTurret;
public class ThreeBall implements PathInterface{

    PathPlannerTrajectory threeBall = PathPlanner.loadPath("3Ball", 6, 4);
    //PathPlannerTrajectory threeBall1 = PathPlanner.loadPath("3Ball1", 6, 4);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; 
    public static final double y = 0.301625; 
    private final DriveSubsystem swerve = RobotContainer.drive;
    private final IndexerSubsystem indexerSubsystem = RobotContainer.indexer;
    private final IntakeSubsystem intakeSubsystem = RobotContainer.intake;
    private final ShooterSubsystem shooterSubsystem = RobotContainer.shooter;
    private final ClimberSubsystem climberSubsystem = RobotContainer.climber;
    PathPlannerState threeBallState = new PathPlannerState();
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

    @Override
    public Command getAutoCommand(){

        threeBallState = threeBall.getInitialState();
        Pose2d threeBallPose = new Pose2d(threeBallState.poseMeters.getTranslation(), threeBallState.holonomicRotation);

        var thetaController = new ProfiledPIDController(10, 0.0, 0.0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var thetaControllerNoRotation = new ProfiledPIDController(0, 0.0, 0.0, kThetaControllerConstraints);
        thetaControllerNoRotation.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand threeBallController = new SwerveControllerCommand(threeBall,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController,
        swerve::setModuleStates, swerve);

        // SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(threeBall1,
        // swerve::getPose, // Functional interface to feed supplier
        // kDriveKinematics, new PIDController(2, 0.0, 0.0), new PIDController(2, 0.0, 0.0), thetaController,
        // swerve::setModuleStates, swerve);
    
        Command setOffset = new SetOffset(swerve, shooterSubsystem, 90);

        Command resetOdo3Ball = new ResetOdometry(swerve, threeBallPose);

        Command intake2Balls = new DriveToBall(intakeSubsystem, indexerSubsystem, swerve, 0.4, 5000);
        Command intake1Ball = new DriveToOneBall(intakeSubsystem, indexerSubsystem, swerve, 0.4, 5000);

        Command pointTurretWall = new PointTurret(shooterSubsystem, 0);
        Command pointTurret3Ball = new PointTurret(shooterSubsystem, 0);

        Command climb = new AutonClimber(climberSubsystem, -25);

        Command stop1 = new Stop(swerve);
        Command stop2 = new Stop(swerve);

        Command spin1 = new AutonSpinUp(shooterSubsystem);
        Command spin2 = new AutonSpinUp(shooterSubsystem);

        Command alignShoot1 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        Command alignShoot2 = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);

        ParallelDeadlineGroup intakeWallBallGroup = new ParallelDeadlineGroup(intake2Balls, intake2Balls, pointTurretWall, climb);
        ParallelDeadlineGroup intake3BallGroup = new ParallelDeadlineGroup(intake1Ball, intake1Ball, pointTurret3Ball);

        ParallelDeadlineGroup spinGroup1 = new ParallelDeadlineGroup(stop1, stop1, spin1);
        ParallelDeadlineGroup spinGroup2 = new ParallelDeadlineGroup(stop2, stop2, spin2);

        SequentialCommandGroup fiveBall = new SequentialCommandGroup(
                            setOffset, resetOdo3Ball, intakeWallBallGroup, spinGroup1, alignShoot1, //Wall ball and shoot 
                            threeBallController, intake3BallGroup, spinGroup2, alignShoot2); //Three ball and shoot
        return fiveBall;
    }
}
