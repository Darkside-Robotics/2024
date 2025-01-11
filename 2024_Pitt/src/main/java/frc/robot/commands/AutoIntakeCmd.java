package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoIntakeCmd extends Command {
    static ShooterSubsystem shooterSubsystem;
    boolean finished;
    //DigitalInput beamBreak = new DigitalInput(0);


    public AutoIntakeCmd(ShooterSubsystem shooterSubsystem_in){
        shooterSubsystem = shooterSubsystem_in;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!isFinished()) {
            shooterSubsystem.runIntake();
            if (shooterSubsystem.noteloaded()) {
                end(true);
                finished = true;
            }
        }
        else{
            shooterSubsystem.stopIntake();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
    
}
