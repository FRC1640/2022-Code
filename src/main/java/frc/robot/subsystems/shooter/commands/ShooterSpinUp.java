package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterSpinUp extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexer;
    Joystick joystick = new Joystick(0);
    Joystick opJoystick = new Joystick(1);
    JoystickButton leftBumper = new JoystickButton(joystick, 5);
    JoystickButton rightBumper = new JoystickButton(joystick, 6);
    JoystickButton oLB = new JoystickButton(opJoystick, 5);
    

    /**
     * Spins up the motor to certain speed
     * @param shooterSubsystem
     */
    public ShooterSpinUp(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexer) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexer = indexer;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {

        shooterSubsystem.setTargetRpm(0); //1750 prime, 1500 deux
        shooterSubsystem.setMotorSpeeds(0.3, 0.3);

        //move shooter head
        // shooterSubsystem.motionProfileAngle(-7); //-3
        // shooterSubsystem.pointTurretPosition(240);
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return (indexer.getProx2() || leftBumper.get() || rightBumper.get() || oLB.get());
        // if(shooterSubsystem.atTargetRpm()){
        // return true;
        // }
        // else{
        //     return false;
        // }
    }
}
