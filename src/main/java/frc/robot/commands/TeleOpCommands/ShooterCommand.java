package frc.robot.commands.TeleOpCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamepadConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private GenericHID controller;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, GenericHID controller){
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;
    }
    public void initialize(){

    }
    public void execute(){
        shooterSubsystem.setPower(0.75);
    }
    public void end(boolean interupted){
        shooterSubsystem.shutdown();
    }
    public boolean isFinished(){
        return !controller.getRawButton(GamepadConstants.kRightBumperPort);
    }  
}
