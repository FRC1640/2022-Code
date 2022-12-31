package frc.robot.subsystems.climber.commands.auton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class LowerClimberAuton1 extends CommandBase{

    private ClimberSubsystem climberSubsystem;
    private double initEncoder = 0;
    private XboxController controller = new XboxController(1);
    private boolean cancel = false;

    public LowerClimberAuton1(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        initEncoder = climberSubsystem.getPosition();
    }

    @Override
    public void execute() {
        if (climberSubsystem.getPosition() <= -30){
            climberSubsystem.setMotors(1.0);
        }
        else{
            climberSubsystem.setMotors(0);
        }

        if(climberSubsystem.getPosition() >= initEncoder + 25 && climberSubsystem.getPosition() <= -35) {
            climberSubsystem.setVertical(false);
        }
        else if(climberSubsystem.getPosition() >= -45) {
            climberSubsystem.setVertical(true);
        } else {
            climberSubsystem.setVertical(true);
        }

        if (climberSubsystem.getPosition() >= -80) {
            climberSubsystem.disengageBrake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotors(0.0);
        climberSubsystem.quickSetup();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.getPosition() >= -30 || cancel;
    }
}
