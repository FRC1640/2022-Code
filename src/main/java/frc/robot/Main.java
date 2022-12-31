package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.Logger;

public final class Main {
  private Main() {}
  
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
