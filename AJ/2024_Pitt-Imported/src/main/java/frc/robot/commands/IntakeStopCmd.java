package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStopCmd extends Command {


 
    static ShooterSubsystem shooterSubsystem;

    public IntakeStopCmd(ShooterSubsystem shooterSubsystem_in){
        shooterSubsystem = shooterSubsystem_in;
    }




    @Override
    public void initialize() {
        shooterSubsystem.stopIntake();
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    
}
