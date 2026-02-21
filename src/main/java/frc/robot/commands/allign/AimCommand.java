package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AimCommand extends Command{
     double targetAngle;

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf

   public static double AimMath(Pose2d estimatedPose) {

          Translation2d hubTranslation = new Translation2d(4.623, 4.034);
          Transform2d shooterOffset = new Transform2d(-0.08, 0.073, new Rotation2d(-Math.PI/2));

          Translation2d shooterPosition = estimatedPose.getTranslation().plus(shooterOffset.getTranslation());
          Translation2d shooterToHub = hubTranslation.minus(shooterPosition);    
          Rotation2d targetAngle = shooterToHub.getAngle().minus(shooterOffset.getRotation());

          return targetAngle.getDegrees();
     }

     public Command Aim (SwerveSubsystem swerveSubsystem, Pose2d estimatedPose, BooleanSupplier cancelCondition){
          targetAngle = AimMath(estimatedPose);
          return new RotateToAngleCommand(swerveSubsystem, targetAngle, cancelCondition);
     }
    
}
