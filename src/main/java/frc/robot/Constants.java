// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.util.AlignUtil;
import frc.robot.util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // ==================== GLOBAL ====================
  public static final String CAN_BUS = "can";

  // ==================== SWERVE ====================

  /** Drive motor constants (physical and electrical) */
  public static class SwerveDriveConstants {
    // Physical Measurements
    public static final double DRIVE_WHEEL_RADIUS = 2.0; // inches
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2.0 * Math.PI * DRIVE_WHEEL_RADIUS); // meters
    public static final double DRIVE_GEAR_REDUCTION = 33.0 / 4.0; // 8.25:1

    // Current Limits (amps)
    // DRIVE_TORQUE_CURRENT_LIMIT: Min=0, Max=120 (Kraken limit), Recommended=60-100
    public static final double DRIVE_TORQUE_CURRENT_LIMIT = 80;
    // DRIVE_SUPPLY_CURRENT_LIMIT: Min=0, Max=120, Recommended=30-60 for brownout prevention
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 40;
    // DRIVE_SUPPLY_CURRENT_LOWER_LIMIT: Should match or be less than SUPPLY_CURRENT_LIMIT
    public static final double DRIVE_SUPPLY_CURRENT_LOWER_LIMIT = 40;
    // DRIVE_SUPPLY_CURRENT_LOWER_TIME: Min=0, Max=5, Recommended=0.5-2.0 seconds
    public static final double DRIVE_SUPPLY_CURRENT_LOWER_TIME = 1.0;

    // Ramp Rate
    // DRIVE_RAMP_RATE: Min=0 (instant), Max=10, Recommended=0-0.5 seconds
    public static final double DRIVE_RAMP_RATE = 0;
  }

  /** Steer motor constants (physical and electrical) */
  public static class SwerveSteerConstants {
    // Physical Measurements
    public static final double STEER_GEAR_REDUCTION = 160.0 / 7.0; // 22.857:1

    // Kraken X44 specs
    public static final double STEER_FREE_SPEED_RPM = 7530.0;

    // Calculated velocities (in CANcoder space: rotations/sec)
    public static final double STEER_MAX_VELOCITY = STEER_FREE_SPEED_RPM / STEER_GEAR_REDUCTION / 60.0; // ~5.49 rot/sec
    public static final double STEER_MAX_ACCELERATION = STEER_MAX_VELOCITY * 10.0; // ~54.9 rot/sec^2

    // MotionMagic settings (rotations/sec and rotations/sec^2)
    public static final double STEER_CRUISE_VELOCITY = STEER_MAX_VELOCITY;
    public static final double STEER_ACCELERATION = STEER_MAX_ACCELERATION;

    // Current Limits (amps)
    // STEER_TORQUE_CURRENT_LIMIT: Min=0, Max=120 (Kraken limit), Recommended=40-80
    public static final double STEER_TORQUE_CURRENT_LIMIT = 80;
    // STEER_SUPPLY_CURRENT_LIMIT: Min=0, Max=120, Recommended=20-40 for brownout prevention
    public static final double STEER_SUPPLY_CURRENT_LIMIT = 30;
    // STEER_SUPPLY_CURRENT_LOWER_LIMIT: Should match or be less than SUPPLY_CURRENT_LIMIT
    public static final double STEER_SUPPLY_CURRENT_LOWER_LIMIT = 30;
    // STEER_SUPPLY_CURRENT_LOWER_TIME: Min=0, Max=5, Recommended=0.25-1.0 seconds
    public static final double STEER_SUPPLY_CURRENT_LOWER_TIME = 0.5;

    // Ramp Rate
    // STEER_RAMP_RATE: Min=0 (instant), Max=10, Recommended=0 for responsive steering
    public static final double STEER_RAMP_RATE = 0;

    // Legacy aliases (for backwards compatibility)
    public static final double STEER_PEAK_CURRENT = STEER_TORQUE_CURRENT_LIMIT;
  }

  /** Swerve module CAN IDs, positions, and PID tuning */
  public static class SwerveConstants {
    // ----- Module CAN IDs -----
    // Front Left
    public static final int FL_DRIVE = 0;
    public static final int FL_STEER = 1;
    public static final int FL_ENCODER = 8;
    public static final double FL_OFFSET = 0;

    // Front Right
    public static final int FR_DRIVE = 2;
    public static final int FR_STEER = 3;
    public static final int FR_ENCODER = 9;
    public static final double FR_OFFSET = 0;

    // Back Left
    public static final int BL_DRIVE = 4;
    public static final int BL_STEER = 5;
    public static final int BL_ENCODER = 10;
    public static final double BL_OFFSET = 0;

    // Back Right
    public static final int BR_DRIVE = 6;
    public static final int BR_STEER = 7;
    public static final int BR_ENCODER = 11;
    public static final double BR_OFFSET = 0;

    // ----- Module Geometry -----
    public static final double MODULE_FRONT_BACK_SPACING = 20.45; // cm between front and back
    public static final double MODULE_LEFTRIGHT_SPACING = 25.45;  // cm between left and right

    // Module positions relative to robot center
    public static final Translation2d FL_POS = new Translation2d( MODULE_FRONT_BACK_SPACING/2.0,  MODULE_LEFTRIGHT_SPACING/2.0);
    public static final Translation2d FR_POS = new Translation2d( MODULE_FRONT_BACK_SPACING/2.0, -MODULE_LEFTRIGHT_SPACING/2.0);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING/2.0,  MODULE_LEFTRIGHT_SPACING/2.0);
    public static final Translation2d BR_POS = new Translation2d(-MODULE_FRONT_BACK_SPACING/2.0, -MODULE_LEFTRIGHT_SPACING/2.0);

    // ----- Speed Limits -----
    // Maximum theoretical velocity: Kraken RPM / gear ratio / 60 * wheel circumference
    // MAX_VEL: Calculated from motor specs, ~4.2 m/s for this configuration
    public static final double MAX_VEL = 6000.0 / SwerveDriveConstants.DRIVE_GEAR_REDUCTION / 60.0 * SwerveDriveConstants.DRIVE_WHEEL_CIRCUMFERENCE; // m/s
    // MAX_OMEGA: Calculated from MAX_VEL and robot geometry
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm(); // rad/s

    // ----- Acceleration & Velocity Limiting (for brownout prevention) -----
    // DEFAULT_VELOCITY_MULTIPLIER: Min=0.0 (stopped), Max=1.0 (full speed), Recommended=0.5-1.0
    public static final double DEFAULT_VELOCITY_MULTIPLIER = 1.0;
    // DEFAULT_ACCEL_LIMIT: Min=0.5, Max=10.0, Recommended=2.0-4.0 m/s² (lower = gentler acceleration, less brownout)
    public static final double DEFAULT_ACCEL_LIMIT = 3.0;
    // DEFAULT_ANGULAR_ACCEL_LIMIT: Min=1.0, Max=20.0, Recommended=4.0-8.0 rad/s² (lower = gentler rotation)
    public static final double DEFAULT_ANGULAR_ACCEL_LIMIT = 6.0;

    // ----- Drive Motor PID (Velocity Control) -----
    public static final double[] DRIVE_P = {9.5, 9.5, 9.5, 9.5};
    public static final double[] DRIVE_I = {0, 0, 0, 0};
    public static final double[] DRIVE_D = {0.1, 0.1, 0.1, 0.1};
    public static final double[] DRIVE_S = {0.5, 0.5, 0.5, 0.5};    // Static friction compensation
    public static final double[] DRIVE_V = {0.12, 0.12, 0.12, 0.12}; // Velocity feedforward

    // ----- Steer Motor PID (Position Control) -----
    public static final double[] STEER_P = {35, 35, 35, 35};
    public static final double[] STEER_I = {0, 0, 0, 0};
    public static final double[] STEER_D = {0.1, 0.1, 0.1, 0.1};
    public static final double[] STEER_S = {0.25, 0.25, 0.25, 0.25}; // Static friction compensation
  }

  // ==================== INTAKE ====================

  public static class IntakeConstants {
    // Roller Motor
    public static final int ROLLER_CAN_ID = 14;
    public static final double ROLLER_IN_SPEED = 0.5;
    public static final double ROLLER_OUT_SPEED = -0.5;
    public static final double ROLLER_CURRENT_LIMIT = 100.0;
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 120.0;
    public static final double ROLLER_OPEN_LOOP_RAMP = 0.05;
    public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

    // Pivot Motor
    public static final int PIVOT_MOTOR_ID = 12;
    public static final double MANUAL_PIVOT_SPEED = 0.15;
    public static final double PIVOT_STATOR_CURRENT_LIMIT = 40.0;
    public static final boolean PIVOT_STATOR_CURRENT_LIMIT_ENABLE = true;

    // Limit Switches
    public static final int TOP_LIMIT_SWITCH_DIO = 0;
    public static final int BOTTOM_LIMIT_SWITCH_DIO = 1;
    public static final Angle TOP_LIMIT = Rotations.of(0.25);
    public static final Angle BOTTOM_LIMIT = Rotations.of(-0.1);

    // CANdle
    public static final int CANDLE_ID = 13;
  }

  // ==================== HOPPER ====================

  public static class HopperConstants {
    public static final int KRAKEN_CAN_ID = 15;

    // Current Limits
    public static final int SUPPLY_CURRENT_LIMIT = 80;
    public static final int STATOR_CURRENT_LIMIT = 60;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;

    // Ramp Rates
    public static final int VOLTAGE_COMPENSATION = 12;
    public static final double OPEN_LOOP_RAMP = 0.5;
    public static final double DUTY_CYCLE_OPEN_LOOP_RAMP = 0.05;

    public static final InvertedValue HOPPERINVERTED = InvertedValue.CounterClockwise_Positive;
  }

  // ==================== VISION ====================

  public static class VisionConstants {
    // Field Dimensions
    public static final double FIELD_X = 16.54;
    public static final double FIELD_Y = 8.07;
    public static final double ROBOT_RADIUS = 0.762;

    // Standard Deviation Data (for pose estimation)
    public static final double[] STD_DEV_DIST = {0.75, 1.00, 1.3, 1.69, 2., 2.51, 2.78, 3.07, 3.54, 4.1, 4.52};
    public static final double[] X_STD_DEV = {0.002, 0.005, 0.007, 0.014, 0.029, 0.074, 0.101, 0.12, 0.151, 0.204, 0.287};
    public static final double[] Y_STD_DEV = {0.002, 0.005, 0.013, 0.020, 0.067, 0.080, 0.095, 0.160, 0.206, 0.259, 0.288};
    public static final double[] O_STD_DEV = {0.002, 0.004, 0.005, 0.011, 0.031, 0.4, 1.72, 1.89, 2.05, 2.443, 2.804};

    // Camera Configurations
    public static final CameraConfig[] cameraConfigs = {
      new CameraConfig(
        "7",
        new Transform3d(0.28, 0, 0, new Rotation3d(0, -Math.toRadians(5), 0)),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    };

    // Regression Models
    public static final PolynomialRegression xStdDevModel = new PolynomialRegression(STD_DEV_DIST, X_STD_DEV, 2);
    public static final PolynomialRegression yStdDevModel = new PolynomialRegression(STD_DEV_DIST, Y_STD_DEV, 2);
    public static final PolynomialRegression oStdDevModel = new PolynomialRegression(STD_DEV_DIST, O_STD_DEV, 1);
  }

  // ==================== AUTONOMOUS / ALIGNMENT ====================

  public static class RotateToAngleConstants {
    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0005;
    public static final double TOLERANCE_DEGREES = 2.0;
  }

  public static class AlignToHubConstants {
    public static final Translation2d HUB_POSITION = new Translation2d(12.51204, 4.03479);
  }

  public static class AlignConstants {
    // Distance tolerance for alignment
    public static final double distanceTolerance = 0.47;

    // ----- Reef Alignment Paths -----
    public static final String reefName = "reefAlignPath";
    public static final String A_alignName = "A align";
    public static final String B_alignName = "B align";
    public static final String C_alignName = "C align";
    public static final String D_alignName = "D align";
    public static final String E_alignName = "E align";
    public static final String F_alignName = "F align";
    public static final String G_alignName = "G align";
    public static final String H_alignName = "H align";
    public static final String I_alignName = "I align";
    public static final String J_alignName = "J align";
    public static final String K_alignName = "K align";
    public static final String L_alignName = "L align";

    public static final List<String> reefPathList = List.of(
      A_alignName, B_alignName, C_alignName, D_alignName, E_alignName, F_alignName,
      G_alignName, H_alignName, I_alignName, J_alignName, K_alignName, L_alignName
    );

    public static final List<Pose2d> blueLeftReefPoseList = List.of(
      AlignUtil.getAlignPath(A_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> blueRightReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> redLeftReefPoseList = List.of(
      AlignUtil.getAlignPath(A_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).flipPath().getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> redRightReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).flipPath().getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> allReefPoseList = List.of(
      AlignUtil.getAlignPath(B_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(C_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(D_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(E_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(F_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(G_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(H_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(I_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(J_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(K_alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(L_alignName).getStartingHolonomicPose().get()
    );

    public static final List<ChassisSpeeds> reefdirectionList = List.of(
      new ChassisSpeeds(-0.5, 0, 0),
      new ChassisSpeeds(-0.25, -0.25, 0),
      new ChassisSpeeds(0.25, -0.25, 0),
      new ChassisSpeeds(0.5, 0, 0),
      new ChassisSpeeds(0.25, 0.25, 0),
      new ChassisSpeeds(-0.25, 0.25, 0)
    );

    // ----- Source Alignment Paths -----
    public static final String sourceName = "sourceAlignPath";
    public static final String LS_1alignName = "LS align 1";
    public static final String LS_2alignName = "LS align 2";
    public static final String LS_3alignName = "LS align 3";
    public static final String RS_1alignName = "RS align 1";
    public static final String RS_2alignName = "RS align 2";
    public static final String RS_3alignName = "RS align 3";

    public static final List<String> sourcePathList = List.of(LS_2alignName, RS_2alignName);

    public static final List<Pose2d> blueSourcePoses = List.of(
      AlignUtil.getAlignPath(LS_2alignName).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(RS_2alignName).getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> redSourcePoses = List.of(
      AlignUtil.getAlignPath(LS_2alignName).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(RS_2alignName).flipPath().getStartingHolonomicPose().get()
    );

    // ----- Algae Alignment Paths -----
    public static final String algae_1 = "Algae Align 1";
    public static final String algae_2 = "Algae Align 2";
    public static final String algae_3 = "Algae Align 3";
    public static final String algae_4 = "Algae Align 4";
    public static final String algae_5 = "Algae Align 5";
    public static final String algae_6 = "Algae Align 6";

    public static final List<String> algaeAlignNames = List.of(
      algae_1, algae_2, algae_3, algae_4, algae_5, algae_6
    );

    public static final List<Pose2d> blueAlgaeAlignPoses = List.of(
      AlignUtil.getAlignPath(algae_1).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_2).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_3).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_4).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_5).getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_6).getStartingHolonomicPose().get()
    );

    public static final List<Pose2d> redAlgaeAlignPoses = List.of(
      AlignUtil.getAlignPath(algae_1).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_2).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_3).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_4).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_5).flipPath().getStartingHolonomicPose().get(),
      AlignUtil.getAlignPath(algae_6).flipPath().getStartingHolonomicPose().get()
    );
  }

  // ==================== LOGGING / DEBUG ====================

  public static class LoggingConstants {
    public static final String SWERVE_TABLE = "SwerveStats";
    public static final String SENSOR_TABLE = "Sensors";
  }

  public static class DebugConstants {
    public static final boolean MASTER_DEBUG = true;
    public static final boolean DRIVE_DEBUG = true;
    public static final boolean STEER_DEBUG = true;
    public static final boolean STATE_DEBUG = true;
  }
}
