package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;

public class IntakeSetRPMCommand extends Command {
    private final RollerIntakeSubsystem intakeSubsystem;

    public IntakeSetRPMCommand(RollerIntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.spinAtTargetRPM();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
