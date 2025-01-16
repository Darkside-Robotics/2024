package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingKook;

public class ClimbingKookUp extends Command{
    static ClimbingKook climbingkook;

    public ClimbingKookUp(ClimbingKook ClimbingKook_in){
        climbingkook = ClimbingKook_in;
    }



    @Override
    public void initialize() {
        climbingkook.ClimbUp();
        
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

