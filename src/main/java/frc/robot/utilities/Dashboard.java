package frc.robot.utilities;

import java.nio.channels.ShutdownChannelGroupException;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.ToggleSolenoid;

public class Dashboard {
    ShuffleboardTab tab = Shuffleboard.getTab("Drive"); 
    // private NetworkTableEntry maxSpeed =
    //     tab.add("Max Speed", 1)
    //         .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
    //         .getEntry();  
    public void run(){
         
        //RobotContainer.intake.setSpeed(maxSpeed.getDouble(0.0));
        SmartDashboard.putData("Intake", new ToggleSolenoid(RobotContainer.intake));
        
        //SmartDashboard.putData("IntakeMotor", new RobotContainer().intake.setSpeed(speed););
    }
}