package frc.robot;

import java.util.function.BooleanSupplier;

import javax.naming.PartialResultException;

import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;

import org.ejml.equation.ParseError;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.commands.SetOffset;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autonomous.paths.DefensiveFourBall;
import frc.robot.autonomous.paths.FiveBall;
import frc.robot.autonomous.paths.FiveBallRed;
import frc.robot.autonomous.paths.MoveForward;
import frc.robot.autonomous.paths.OneBall;
import frc.robot.autonomous.paths.PathInterface;
import frc.robot.autonomous.paths.TestOneMeter;
import frc.robot.autonomous.paths.ThreeBall;
import frc.robot.autonomous.paths.ThreeBallNew;
import frc.robot.autonomous.paths.TwoBall;
import frc.robot.autonomous.paths.TwoBallMirrored;
import frc.robot.autonomous.paths.TwoBallOther;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.ExtendClimber;
import frc.robot.subsystems.climber.commands.auton.HighClimberCommand;
import frc.robot.subsystems.climber.commands.auton.LowerClimberAuton;
import frc.robot.subsystems.climber.commands.auton.TraversalClimberCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveToBall;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.drive.commands.Rotate;
import frc.robot.subsystems.drive.commands.SetOffset;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.commands.Index;
import frc.robot.subsystems.indexer.commands.IndexShooter;
import frc.robot.subsystems.indexer.commands.IndexShooterFast;
import frc.robot.subsystems.indexer.commands.Outdex;
import frc.robot.subsystems.indexer.commands.SetIndexerSpeed;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.Intake;
import frc.robot.subsystems.intake.commands.Outtake;
import frc.robot.subsystems.intake.commands.SetIntakeSpeed;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.AlignTurret;
import frc.robot.subsystems.shooter.commands.CalculatedShot;
import frc.robot.subsystems.shooter.commands.ClimberRotateTurret;
import frc.robot.subsystems.shooter.commands.FenderShot;
import frc.robot.subsystems.shooter.commands.LowFender;
import frc.robot.subsystems.shooter.commands.ShooterSpinUp;
import frc.robot.subsystems.shooter.commands.SpinFender;
public class RobotContainer {

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  Joystick joystick = new Joystick(0);
  Joystick opJoystick = new Joystick(1);

  public static final DriveSubsystem drive = new DriveSubsystem();
  public static final IndexerSubsystem indexer = new IndexerSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final ClimberSubsystem climber = new ClimberSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem();
  private SendableChooser<PathInterface> sChooser;
  public boolean climbing;
  public RobotContainer() {
    
    configureButtonBindings();
    sChooser = new SendableChooser<PathInterface>();
    sChooser.addOption("Five Ball Blue", new FiveBall());
    sChooser.addOption("Five Ball Red", new FiveBallRed());
    sChooser.addOption("Three Ball", new ThreeBallNew());
    sChooser.addOption("Two Ball", new TwoBall());
    sChooser.addOption("Two Ball Other", new TwoBallOther());
    sChooser.addOption("Reset", new TestOneMeter());
    Shuffleboard.getTab("Auton").add(sChooser).withSize(5, 5).withPosition(0, 0);
    Shuffleboard.getTab("Drive").addBoolean("Lower Sensor", () -> !intake.getProx1()).withPosition(0, 2).withSize(2, 2);
    Shuffleboard.getTab("Drive").addBoolean("Upper sensor", () -> !indexer.getProx2()).withPosition(0, 0).withSize(2, 2);
    Shuffleboard.getTab("Drive").addBoolean("Left turret limit", () -> shooter.turretLimit1()).withPosition(2, 0).withSize(1, 6);
    Shuffleboard.getTab("Drive").addBoolean("Right turret limit", () -> shooter.turretLimit0()).withPosition(3, 0).withSize(1, 6);
    Shuffleboard.getTab("Drive").addCamera("Camera", "USB Camera 0", "10.16.40.2:5802").withPosition(4, 0).withSize(6, 3);
    Shuffleboard.getTab("Drive").addCamera("Limelight", "Limelight", "10.16.40.58:5800").withPosition(4, 3).withSize(6, 3);
    drive.initDefaultCommand();
    climber.initDefaultCommand();
    shooter.setDefaultCommand(new AlignTurret(shooter, drive));
    
  }

  public void configureButtonBindings() {

    JoystickButton startButton = new JoystickButton(joystick, 8);
    JoystickButton oStartButton = new JoystickButton(opJoystick, 8);
    JoystickButton opSelectButton = new JoystickButton(opJoystick, 7);
    startButton.whenPressed(new ResetGyro(drive, shooter));
    Command climberRotateTurret = new ClimberRotateTurret(shooter, climber);
    opSelectButton.whenPressed(climberRotateTurret);
    opSelectButton.whenReleased(() -> climberRotateTurret.cancel());
    JoystickButton xButton = new JoystickButton(joystick, 3);
    xButton.whenHeld(new DriveToBall(intake, indexer, drive, 0.4, 99999999));
    Button rightTrigger = new Button() {
            
      public boolean get() {
          return driverController.getRightTriggerAxis() > 0.1;
      }

  };

  Button leftTrigger = new Button() {
      
      public boolean get() {
          return driverController.getLeftTriggerAxis() > 0.1;
      }

  };


    //Stop intake if both sensors are triggered
  JoystickButton oLB = new JoystickButton(opJoystick, 5);
  JoystickButton oRB = new JoystickButton(opJoystick, 6);

  oRB.whenHeld(new SpinFender(shooter));
  Button stopIntake = new Button() {
  public boolean get() {
      if (!intake.getProx1() && !indexer.getProx2()) //prox1 lower prox2 higher
      {
        return false;
      }
      else if (!rightTrigger.get())
      {
        return false;
      }
      else if (rightTrigger.get()){
        return true;
      }
      else {
        return true;
      }

     
  }
  };
  Button runIndexer = new Button() {
    public boolean get() {
        return !intake.getProx1();
    }
  };
  JoystickButton leftBumper = new JoystickButton(joystick, 5);
  JoystickButton rightBumper = new JoystickButton(joystick, 6);
  Button shooterTrigger = new Button() {

    @Override
    public boolean get() {
        return shooter.atTargetRpm() && shooter.atTargetAngle() && shooter.pointedAtTarget();//&& shooter.pointedAtTarget();
    }
  };
  Button areaTrigger = new Button(){
    @Override
    public boolean get(){
      return shooter.getArea() != 0 && rightBumper.get(); 
    }
  };

  Button fastTrigger = new Button() {

    @Override
    public boolean get() {
        return shooter.atTargetAngle() && oLB.get();
    }

  };

  JoystickButton aButton = new JoystickButton(joystick, 1);
  JoystickButton oAbutton = new JoystickButton(opJoystick, 1);
  JoystickButton oYbutton = new JoystickButton(opJoystick, 4);
  SequentialCommandGroup climbGroup = new SequentialCommandGroup(new LowerClimberAuton(climber), new ExtendClimber(climber, -15));

  SequentialCommandGroup traversalClimbGroup = new SequentialCommandGroup(new TraversalClimberCommand(climber, shooter));
  oYbutton.whenPressed(traversalClimbGroup);
  SequentialCommandGroup highClimbGroup = new SequentialCommandGroup(new HighClimberCommand(climber, shooter));
  oAbutton.whenPressed(highClimbGroup);
    oStartButton.whenPressed(() -> {
      traversalClimbGroup.cancel();
      highClimbGroup.cancel();
    });
  
  Command index = new IndexShooter(indexer, intake);
  ParallelCommandGroup indexShooterGroup = new ParallelCommandGroup(new FenderShot(shooter));
  aButton.whenHeld(new Outdex(indexer, intake));
  shooterTrigger.whenPressed(index);
  
  // if (rightBumper.get() && !areaTrigger.get()){
  //    index.cancel();
  // }
  
  leftBumper.whileHeld(indexShooterGroup);
  leftBumper.whenReleased(() -> {
    index.cancel();
  });

  ParallelCommandGroup indexShooterGroupCalculated = new ParallelCommandGroup(new CalculatedShot(shooter));
  areaTrigger.whileHeld(indexShooterGroupCalculated);
  
  rightBumper.whenReleased(() -> {
    index.cancel();
  });

  Command indexFast = new IndexShooterFast(indexer, intake);
  ParallelCommandGroup indexShooterGroupFast = new ParallelCommandGroup(new LowFender(shooter));
  // fastTrigger.whenPressed(indexFast);
  oLB.whileHeld(indexShooterGroupFast);
  oLB.whenReleased(() -> {
    index.cancel();
  });

  if (leftTrigger.get() == false){
    runIndexer.whenPressed(new Index(indexer));
  }
  
  //Run intake on right trigger and stop it if both sensors are triggered
  Command intakeCommand = new Intake(intake, indexer);
  
  stopIntake.whenPressed(intakeCommand);
  stopIntake.whenReleased(() -> intakeCommand.cancel());
  //ParallelCommandGroup outdexGroup = new ParallelCommandGroup(new Outtake(intake), new Outdex(indexer, intake));
  // JoystickButton rightBumper = new JoystickButton(joystick, 6);
  //rightBumper.whenHeld(outdexGroup);

  Button runShooter = new Button() {

    @Override
    public boolean get() {
        return !indexer.getProx2();
    }

  };
  // runShooter.whenPressed(new ShooterSpinUp(shooter,indexer)); //spins motor when getProx2 has a ball (doesnt angle)

}

  public Command getAutonomousCommand() {
    CommandScheduler.getInstance().clearButtons();
    
    Command autoCommand = sChooser.getSelected().getAutoCommand();

    // Run path following command, then stop at the end.
    return autoCommand.andThen(() -> drive.drive(0, 0, 0, false));
  }
}
