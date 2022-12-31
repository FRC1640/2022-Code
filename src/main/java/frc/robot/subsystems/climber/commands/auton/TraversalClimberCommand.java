package frc.robot.subsystems.climber.commands.auton;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.TiltClimber;
import frc.robot.subsystems.climber.commands.UntiltClimber;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ClimberRotateTurret;

public class TraversalClimberCommand extends CommandBase {
    ParallelCommandGroup climb;
    SequentialCommandGroup climberGroup;
    //Move down to -22, no motion profile at full speed
    //Half speed to limit switch, no motion profile
    //Reverse at half speed until -10 done
    //robot on mid bar, static hooks 
    //Move torwards shooter while extend to max, motion profile done
    //Once less then 30 degrees and greater then 10 degrees, move away from shooter done
    //Move down to -22, no motion profile at full speed done
    //Half speed to limit switch, no motion profile done
    //Reverse at half speed until -10 done
    //robot on high bar, static hooks done
    //Move torwards shooter while extend to max, motion profile done
    //Once less then 30 degrees and greater then 10 degrees, move away from shooter
    //move down to limit, once close half speed
    public TraversalClimberCommand(ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem) {
        addRequirements(climberSubsystem);
       
        Command turret = new ClimberRotateTurret(shooterSubsystem, climberSubsystem);
        Command lower = new LowerClimber(climberSubsystem);
        Command lower1 = new LowerClimberAuton(climberSubsystem);
        Command lower2 = new LowerClimberAuton1(climberSubsystem);
        Command reverse = new ReverseClimber(climberSubsystem);
        Command reverse1 = new ReverseClimber(climberSubsystem);
        Command untiltCommand = new UntiltClimber(climberSubsystem);
        Command driveClimber = new MotionProfileClimber(climberSubsystem, -129, 26, 60);
        Command checkAngle = new AngleCheck(climberSubsystem, 0, 33);
        Command checkAngle2 = new AngleCheck(climberSubsystem, 0, 40);
        Command checkAngle3 = new AngleCheck(climberSubsystem, 0, 40);
        Command untiltCommand1 = new UntiltClimber(climberSubsystem);
        Command driveClimber1 = new MotionProfileClimber(climberSubsystem, -129, 25, 60);
        Command checkAngle1 = new AngleCheck(climberSubsystem, 0, 45);
        Command tilt = new TiltClimber(climberSubsystem);
        Command tilt1 = new TiltClimber(climberSubsystem);
        ParallelCommandGroup extend1 = new ParallelCommandGroup(tilt1, driveClimber1);
        ParallelCommandGroup extend = new ParallelCommandGroup(tilt, driveClimber);
        ParallelCommandGroup lowerGroup = new ParallelCommandGroup(lower);
        climberGroup = new SequentialCommandGroup(
            lowerGroup,
            checkAngle2,
            reverse,
            extend,
            checkAngle,
            untiltCommand,
            lower1,
            checkAngle3,
            reverse1,
            extend1,
            checkAngle1,
            untiltCommand1,
            lower2
        );
        climb = new ParallelCommandGroup(climberGroup, turret);
    }
    
    @Override
    public void initialize() {
        climb.initialize();
    }

    @Override
    public void execute() {
        climb.execute();
        
    }

    @Override
    public void end(boolean interrupted) {
        climb.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return climb.isFinished();
    }
    
}
