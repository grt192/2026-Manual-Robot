// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.VisionConstants;
import frc.robot.commands.allign.AlignToHubCommand;
import frc.robot.commands.allign.RotateToAngleCommand;
// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
// Subsystems
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.Vision.VisionSubsystem;
// import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.PDHSubsystem;
import frc.robot.Constants.PDHConstants;
// import frc.robot.Constants.IntakeConstants;

// Commands
import frc.robot.commands.intake.ManualIntakePivotCommand;

import com.ctre.phoenix6.CANBus;

// import frc.robot.commands.intake.SetIntakePivotCommand;
// import frc.robot.commands.hopper.HopperSetRPMCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;



// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix6.CANBus;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private PS5DriveController driveController;
  private CommandPS5Controller gamer = new CommandPS5Controller(1);

  
  private final CANBus canivore = new CANBus("can");
  private final CANBus swerveCan = new CANBus("swerveCan");

  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveCan);

  //shooter subsystem
  private flywheel wheel = new flywheel(canivore);
  private hood hooded = new hood(canivore);

  //intake subsystem
  private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(canivore);
  private final PivotIntakeSubsystem pivotIntake = new PivotIntakeSubsystem();
  
  //hopper subsystem
  private final HopperSubsystem HopperSubsystem = new HopperSubsystem(canivore);
  private final PDHSubsystem pdhSubsystem = new PDHSubsystem(PDHConstants.PDH_CAN_ID);
  
  private final Field2d m_field = new Field2d();

  // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
  //   VisionConstants.cameraConfigs[0]
  // );
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

    constructDriveController();
    constructMechController();
    configureBindings();
    configureAutoChooser();
    CameraServer.startAutomaticCapture(); // start driver cam
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named f`actories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

      //gun.run();
      /* Driving -- One joystick controls translation, the other rotation. If the robot-relative button is held down,
      * the robot is controlled along its own axes, otherwise controls apply to the field axes by default. If the
      * swerve aim button is held down, the robot will rotate automatically to always face a target, and only
      * translation will be manually controllable. */

    
    swerveSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(
          driveController.getForwardPower(),
          driveController.getLeftPower(),
          driveController.getRotatePower()
        );
        }, 
        swerveSubsystem
      )
    );
    driveController.getRelativeMode().whileTrue(
      new RunCommand(
        () -> {
          swerveSubsystem.setRobotRelativeDrivePowers(
            driveController.getForwardPower(),
            driveController.getLeftPower(),
            driveController.getRotatePower()
          );
          driveController.getRotatePower();
          }, swerveSubsystem)
    );



    //shooter mech stuff
    Trigger dpadUp = new Trigger(() -> gamer.getHID().getPOV() == 0);
    Trigger dpadDown = new Trigger(() -> gamer.getHID().getPOV() == 180);
    Trigger dpadNeutral = new Trigger(() -> {
      int pov = gamer.getHID().getPOV();
      return pov != 0 && pov != 180;
    });
  
    
    wheel.setDefaultCommand(
      new RunCommand(
          () -> {
            double r2 = gamer.getR2Axis();
            wheel.flySpeed((r2+1)/2);
          },
          wheel));
    

    dpadUp.whileTrue(new RunCommand(() -> hooded.hoodSpeed(0.05), hooded));
    dpadDown.whileTrue(new RunCommand(() -> hooded.hoodSpeed(-0.05), hooded));
    dpadNeutral.onTrue(new RunCommand(() -> hooded.hoodSpeed(0.0), hooded));
          
   

      
  


    /* Pressing the button resets the field axes to the current robot axes. */
    /* 
    driveController.bindDriverHeadingReset(
      () ->{
        swerveSubsystem.resetDriverHeading();
      },
      swerveSubsystem
    );

    // Cancel rotate command if driver touches any stick
    BooleanSupplier driverInput = () ->
        Math.abs(driveController.getForwardPower()) > 0 ||
        Math.abs(driveController.getLeftPower()) > 0 ||
        Math.abs(driveController.getRotatePower()) > 0;
    // --- Intake pivot set-position controls (commented out for now) ---
    // mechController.square().onTrue(
    //   new SetIntakePivotCommand(pivotIntake, IntakeConstants.STOWED_POS)
    // );
    // mechController.cross().onTrue(
    //   new SetIntakePivotCommand(pivotIntake, IntakeConstants.EXTENDED_POS)
    // );

  // circle for the manual hopper
    mechController.square().whileTrue(
      new RunCommand(
        () -> HopperSubsystem.setManualControl(1.0),
        HopperSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> HopperSubsystem.stop(),
        HopperSubsystem
      )
    );

    // --- Hopper RPM control (commented out for now) ---
    // mechController.triangle().onTrue(
    //   new HopperSetRPMCommand(HopperSubsystem)
    // );

    /* Intake Controls - Hold button to run rollers */
    // R1 - intake in
    gamer.R1().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_IN_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      )
    );

    // L1 - intake out
    gamer.L1().whileTrue(
      new RunCommand(
        () -> intakeSubsystem.setDutyCycle(Constants.IntakeConstants.ROLLER_OUT_SPEED),
        intakeSubsystem
      )
    ).onFalse(
      new InstantCommand(
        () -> intakeSubsystem.stop(),
        intakeSubsystem
      )
    );

     // Pivot Configs: R2 for pivot up and L2 for pivot down
        pivotIntake.setDefaultCommand(
    new ManualIntakePivotCommand(pivotIntake, () -> gamer.getR2Axis() - gamer.getL2Axis()
     )
   );

    // Triangle - reset pivot encoder position to 0
  //  mechController.triangle().onTrue(
    //  new InstantCommand(() -> pivotIntake.resetPosition(), pivotIntake)
    //);



  }


  public void updateDashboard() {
    // Robot position
    Pose2d robotPose = swerveSubsystem.getRobotPosition();
    m_field.setRobotPose(robotPose);

    // Match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // D-pad steer speed limiting (scales MotionMagic cruise velocity)
    // Up = 100%, Right = 75%, Down = 50%, Left = 25%
    new Trigger(() -> driveController.getPOV() == 0)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(1.0)));
    new Trigger(() -> driveController.getPOV() == 90)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.75)));
    new Trigger(() -> driveController.getPOV() == 180)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.50)));
    new Trigger(() -> driveController.getPOV() == 270)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.setSteerSpeedLimit(0.25)));
  }
    

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs mech controller
   */
  private void constructMechController(){
  }

  /**
   * Config the autonomous command chooser
   */
  private void configureAutoChooser() {
    // Add auton here
    autoChooser.setDefaultOption("Do Nothing", null);

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
 