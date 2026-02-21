package frc.robot.commands;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.allign.AimCommand;


public class TestAimCommand {
    @Test
    public void testa(){
        Pose2d pose = new Pose2d(4,2,new Rotation2d(100));
        // AimCommand aimCommand = new AimCommand();
        // double result = aimCommand.Aim();
        double result = AimCommand.AimMath(pose);
        System.out.println(result);
        
    }
    
    @Test
    public void testb(){
        Pose2d pose = new Pose2d(2,4,new Rotation2d(0));
        // AimCommand aimCommand = new AimCommand();
        // double result = aimCommand.Aim();
        double result = AimCommand.AimMath(pose);
        System.out.println(result);
        
    }
    
}
