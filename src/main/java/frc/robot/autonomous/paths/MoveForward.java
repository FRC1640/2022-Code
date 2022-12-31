package frc.robot.autonomous.paths;
import java.sql.ClientInfoStatus;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
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
public class MoveForward implements PathInterface{
    PathPlannerTrajectory testPath = PathPlanner.loadPath("Drive2", 6, 4);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; // 10.875"
    public static final double y = 0.301625; // 11.875"
    private final DriveSubsystem swerve = RobotContainer.drive;
    private final IndexerSubsystem indexerSubsystem = RobotContainer.indexer;
    private final IntakeSubsystem intakeSubsystem = RobotContainer.intake;
    private final ShooterSubsystem shooterSubsystem = RobotContainer.shooter;
    private final ClimberSubsystem climberSubsystem = RobotContainer.climber;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
    PathPlannerState state = new PathPlannerState();

    @Override
    public Command getAutoCommand(){

        state = testPath.getInitialState();
        Pose2d startingPose = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        swerve.resetOdometry(startingPose);

        var thetaController1 = new ProfiledPIDController(50, 0.0, 0.0, kThetaControllerConstraints);
        thetaController1.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(testPath,
                swerve::getPose, // Functional interface to feed supplier
                kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), thetaController1,
                swerve::setModuleStates, swerve);
    
        Command pointWheels = new Stop(swerve);
        Command setOffset = new SetOffset(swerve, shooterSubsystem, 270); 
        SequentialCommandGroup oneBall = new SequentialCommandGroup(setOffset, swerveControllerCommand, pointWheels);
        return oneBall;
    }
}
