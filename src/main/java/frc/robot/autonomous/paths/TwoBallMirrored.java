package frc.robot.autonomous.paths;
import java.sql.ClientInfoStatus;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.ExtendClimber;
import frc.robot.subsystems.climber.commands.UntiltClimber;
import frc.robot.subsystems.climber.commands.auton.AutonClimber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveToBall;
import frc.robot.subsystems.drive.commands.PointWheels;
import frc.robot.subsystems.drive.commands.Rotate;
import frc.robot.subsystems.drive.commands.SetOffset;
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.Intake;
import frc.robot.subsystems.intake.commands.IntakeAuto;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.Align;
import frc.robot.subsystems.shooter.commands.MultiShot;
import frc.robot.subsystems.shooter.commands.OneShot;
import frc.robot.subsystems.shooter.commands.OneShotAlign;
import frc.robot.subsystems.shooter.commands.PointTurret;
public class TwoBallMirrored implements PathInterface{
    // PathPlannerTrajectory testPath = PathPlanner.loadPath("2Ball", 1, 3);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; // 10.875"
    public static final double y = 0.301625; // 11.875"
    private final DriveSubsystem swerve = RobotContainer.drive;
    private final IndexerSubsystem indexerSubsystem = RobotContainer.indexer;
    private final IntakeSubsystem intakeSubsystem = RobotContainer.intake;
    private final ShooterSubsystem shooterSubsystem = RobotContainer.shooter;
    private final ClimberSubsystem climberSubsystem = RobotContainer.climber;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
    @Override
    public Command getAutoCommand(){

        // TrajectoryConfig config = new TrajectoryConfig(3.0, 1.0)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(kDriveKinematics);
        // config.setReversed(false);

        // TrajectoryConfig config2 = new TrajectoryConfig(3.0, 1.0)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(kDriveKinematics);
        // config2.setReversed(true);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(0.125, 0.0)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(0.25, 0.0, new Rotation2d(0)), config);

        // Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(-0.1, 0.0)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(-0.2, 0.0, new Rotation2d(Math.toRadians(200))), config);

        // var thetaController = new ProfiledPIDController(-0.075, 0.0, 0.0, kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // var thetaController2 = new ProfiledPIDController(3, 0.0, 0.0, kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
        //         swerve::getPose, // Functional interface to feed supplier
        //         kDriveKinematics, new PIDController(0.45, 0.0, 0.0), new PIDController(3, 0.0, 0.0), thetaController,
        //         swerve::setModuleStates, swerve);

        // SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(exampleTrajectory2,
        //         swerve::getPose, // Functional interface to feed supplier
        //         kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController2,
        //         swerve::setModuleStates, swerve);
    
        Command pointWheels = new Stop(swerve);
        Command pointWheels2 = new PointWheels(swerve, 0);
        Command climb = new AutonClimber(climberSubsystem, -25);
        Command align = new OneShotAlign(shooterSubsystem, intakeSubsystem, indexerSubsystem);
        Command pointTurret1 = new PointTurret(shooterSubsystem, 0); //163 == back
        Command intake = new DriveToBall(intakeSubsystem, indexerSubsystem, swerve, 0.25, 5000);
        Command setOffset = new SetOffset(swerve, shooterSubsystem, 224.5); 
        ParallelDeadlineGroup group = new ParallelDeadlineGroup(intake, intake, pointTurret1, climb);
        SequentialCommandGroup group2 = new SequentialCommandGroup(setOffset, group, pointWheels, align);
        // swerve.resetOdometry(exampleTrajectory2.getInitialPose()); 
        return group2;
    }
}
