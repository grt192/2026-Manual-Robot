package frc.robot.commands.allign;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class TestAimCommand {
    @Test
    public void testing(){
        Pose2d pose = new Pose2d(2,3,new Rotation2d(3));
        // AimCommand aimCommand = new AimCommand();
        // double result = aimCommand.Aim();
        double result = AimCommand.AimMath(pose);
        System.out.println(result);
        
    }
    
    @Test
    public void testing(){
        Pose2d pose = new Pose2d(4,5,new Rotation2d(8));
        // AimCommand aimCommand = new AimCommand();
        // double result = aimCommand.Aim();
        double result = AimCommand.AimMath(pose);
        System.out.println(result);
        
    }
    
}
