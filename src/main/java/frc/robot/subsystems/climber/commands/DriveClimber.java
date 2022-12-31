package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class DriveClimber extends CommandBase {

    ClimberSubsystem climberSubsystem;
    XboxController opController = new XboxController(1);

    public DriveClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //System.out.println("Limit: " + climberSubsystem.limit());
        //System.out.println(climberSubsystem.getPosition());
        //System.out.println(climberSubsystem.getGyroRoll());
        // if (climberSubsystem.getGyroRoll() >= 5 && climberSubsystem.getGyroRoll() <= 10)
        // {
            // System.out.println("left y " + opController.getLeftY());
            if(opController.getRightY() < 0.3) {
                climberSubsystem.setMotors(opController.getRightY()*1.0);
            }
            else if (opController.getRightY() > -0.3) {
                climberSubsystem.setMotors(opController.getRightY()*1.0);
            } else {
                climberSubsystem.setMotors(0.0);
            }

            // System.out.println(climberSubsystem.getPosition());

            if(opController.getBButton()) {
                climberSubsystem.setVertical(true);
            }
            if(opController.getXButton()) {
                climberSubsystem.setVertical(false);
            }
            // if(opController.getAButton()) {
            //     climberSubsystem.engageBrake();
            // }
            // if(opController.getYButton()) {
            //     climberSubsystem.disengageBrake();
            // }
        // }
        Button left = new Button() {
            public boolean get() {
                return opController.getPOV() == 90;
            }
        };

        Button right = new Button() {
            public boolean get() {
                return opController.getPOV() == 270;
            }
        };
        if (left.get()){
            climberSubsystem.engageBrake();
        }
        if (right.get()){
            climberSubsystem.disengageBrake();
        }

        // System.out.println(climberSubsystem.limit());
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
