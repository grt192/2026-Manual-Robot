package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AimCommand extends Command{
    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf

   public static double AimMath(Pose2d estimatedPose) {
        Transform2d fieldToHub = new Transform2d(4.623,4.034, new Rotation2d());
        Transform2d robotToShooter = new Transform2d(2,3,new Rotation2d(Math.PI/2));
        Transform2d fieldToRobot = new Transform2d(estimatedPose.getTranslation(), estimatedPose.getRotation());

        Transform2d shooterToHub = robotToShooter.inverse()
        .plus(fieldToRobot.inverse())
        .plus(fieldToHub);

        Double targetAngle = shooterToHub.getRotation().getDegrees();
        return targetAngle;
        // new RotateToAngleCommand(swerveSubsystem, targetAngle, cancelCondition).schedule();
   }
    
}
