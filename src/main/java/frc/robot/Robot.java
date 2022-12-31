package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.Dashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  StringLogEntry commandLog;
  private RobotContainer m_robotContainer;
  private Dashboard dashboard = new Dashboard();
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    // DataLogManager.start();
    // DataLog log = DataLogManager.getLog();
    // commandLog = new StringLogEntry(log, "/commandLogs");
    // CommandScheduler.getInstance()

    //   .onCommandInitialize(

    //       command ->

    //           commandLog.append("Command init: " + command.getName()));

    // CommandScheduler.getInstance()

    //     .onCommandInterrupt(

    //         command ->

    //           commandLog.append("Command interrupt: " + command.getName()));

    // CommandScheduler.getInstance()

    //     .onCommandFinish(

    //         command ->

    //             commandLog.append("Command finish: " + command.getName()));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().clearButtons();
    m_robotContainer.configureButtonBindings();
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    dashboard.run();
  }

  @Override
  public void testPeriodic() {}
}
