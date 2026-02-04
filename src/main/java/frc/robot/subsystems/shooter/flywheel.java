package frc.robot.subsystems.shooter;

import frc.robot.Constants.FlywheelConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    // Status signals for logging
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // Logging prefix
    private static final String LOG_PREFIX = "Shooter/Flywheel/";

    // Track commanded values
    private double commandedDutyCycle = 0.0;

    public flywheel(CANBus cn) {
        upperMotor = new TalonFX(FlywheelConstants.MOTOR_ID, cn);

        // Initialize status signals
        velocitySignal = upperMotor.getVelocity();
        positionSignal = upperMotor.getPosition();
        supplyVoltageSignal = upperMotor.getSupplyVoltage();
        motorVoltageSignal = upperMotor.getMotorVoltage();
        statorCurrentSignal = upperMotor.getStatorCurrent();
        supplyCurrentSignal = upperMotor.getSupplyCurrent();
        temperatureSignal = upperMotor.getDeviceTemp();

        // Set update frequency for all signals (optimize CAN bus usage)
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, // 50 Hz update rate
            velocitySignal,
            positionSignal,
            supplyVoltageSignal,
            motorVoltageSignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            temperatureSignal
        );

        // Optimize bus utilization
        upperMotor.optimizeBusUtilization();

        config();
    }

    public void config() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Configure current limits
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FlywheelConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(FlywheelConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        cfg.withCurrentLimits(currLim);

        // Configure feedback
        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;
        cfg.withFeedback(feedback);

        upperMotor.getConfigurator().apply(cfg);
    }

    public void flySpeed(double speed) {
        commandedDutyCycle = speed;
        upperMotor.setControl(dutyCycleOut.withOutput(speed));
    }

    public void stop() {
        flySpeed(0.0);
    }

    public double getVelocityRPS() {
        return velocitySignal.refresh().getValueAsDouble();
    }

    public double getVelocityRPM() {
        return getVelocityRPS() * 60.0;
    }

    public boolean isAtSpeed(double targetRPM, double toleranceRPM) {
        return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM;
    }

    @Override
    public void periodic() {
        // Refresh all signals at once for efficiency
        BaseStatusSignal.refreshAll(
            velocitySignal,
            positionSignal,
            supplyVoltageSignal,
            motorVoltageSignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            temperatureSignal
        );

        // Motor state logging
        Logger.recordOutput(LOG_PREFIX + "VelocityRPS", velocitySignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "VelocityRPM", velocitySignal.getValueAsDouble() * 60.0);
        Logger.recordOutput(LOG_PREFIX + "PositionRotations", positionSignal.getValueAsDouble());

        // Electrical logging
        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "MotorVoltage", motorVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps", supplyCurrentSignal.getValueAsDouble());

        // Thermal logging
        Logger.recordOutput(LOG_PREFIX + "TemperatureCelsius", temperatureSignal.getValueAsDouble());

        // Command logging
        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle", commandedDutyCycle);
        Logger.recordOutput(LOG_PREFIX + "CommandedVoltage", commandedDutyCycle * supplyVoltageSignal.getValueAsDouble());

        // Calculated metrics
        double power = motorVoltageSignal.getValueAsDouble() * statorCurrentSignal.getValueAsDouble();
        Logger.recordOutput(LOG_PREFIX + "PowerWatts", power);

        // Connection status
        Logger.recordOutput(LOG_PREFIX + "Connected", upperMotor.isConnected());
        Logger.recordOutput("Torque", upperMotor.getTorqueCurrent().getValueAsDouble() * upperMotor.getMotorKT().getValueAsDouble());
    }
}