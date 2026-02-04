package frc.robot.subsystems.shooter;

import frc.robot.Constants.HoodConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class hood extends SubsystemBase {

    private final TalonFX hoodMotor;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final CANdi limitSwitch;

    // Status signals for logging
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // Logging prefix
    private static final String LOG_PREFIX = "Shooter/Hood/";

    // Track state
    private double commandedSpeed = 0.0;
    private boolean previousLimitPressed = false;
    private boolean isAtUpperLimit = false;
    private boolean isAtLowerLimit = false;

    public hood(CANBus cn) {
        hoodMotor = new TalonFX(HoodConstants.MOTOR_ID, cn);
        limitSwitch = new CANdi(HoodConstants.LIMIT_SWITCH_ID, cn);

        // Initialize status signals
        velocitySignal = hoodMotor.getVelocity();
        positionSignal = hoodMotor.getPosition();
        supplyVoltageSignal = hoodMotor.getSupplyVoltage();
        motorVoltageSignal = hoodMotor.getMotorVoltage();
        statorCurrentSignal = hoodMotor.getStatorCurrent();
        supplyCurrentSignal = hoodMotor.getSupplyCurrent();
        temperatureSignal = hoodMotor.getDeviceTemp();

        // Set update frequency for all signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            velocitySignal,
            positionSignal,
            supplyVoltageSignal,
            motorVoltageSignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            temperatureSignal
        );

        // Optimize bus utilization
        hoodMotor.optimizeBusUtilization();

        // Initialize hood to starting angle
        hoodMotor.setPosition(HoodConstants.HOME_POSITION);
        config();
    }

    public void config() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        cfg.withFeedback(feedback);

        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        cfg.withCurrentLimits(currLim);

        hoodMotor.getConfigurator().apply(cfg);
    }

    public void hoodSpeed(double speed) {
        commandedSpeed = speed;
        double currentPosition = positionSignal.refresh().getValueAsDouble();

        isAtUpperLimit = currentPosition >= HoodConstants.UPPER_LIMIT && speed > 0;
        isAtLowerLimit = currentPosition <= HoodConstants.LOWER_LIMIT && speed < 0;

        if (isAtUpperLimit || isAtLowerLimit) {
            hoodMotor.setControl(dutyCycleOut.withOutput(0));
        } else {
            hoodMotor.setControl(dutyCycleOut.withOutput(speed));
        }
    }

    public void stop() {
        hoodSpeed(0.0);
    }

    public double getPosition() {
        return positionSignal.refresh().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Handle limit switch homing
        boolean limitPressed = limitSwitch.getS1Closed().refresh().getValue();
        if (limitPressed && !previousLimitPressed) {
            hoodMotor.setPosition(HoodConstants.HOME_POSITION);
        }
        previousLimitPressed = limitPressed;

        // Refresh all signals at once
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
        Logger.recordOutput(LOG_PREFIX + "PositionRotations", positionSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "VelocityRPS", velocitySignal.getValueAsDouble());

        // Electrical logging
        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "MotorVoltage", motorVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps", supplyCurrentSignal.getValueAsDouble());

        // Thermal logging
        Logger.recordOutput(LOG_PREFIX + "TemperatureCelsius", temperatureSignal.getValueAsDouble());

        // Command logging
        Logger.recordOutput(LOG_PREFIX + "CommandedSpeed", commandedSpeed);

        // Limit switch and soft limit logging
        Logger.recordOutput(LOG_PREFIX + "LimitSwitchPressed", limitPressed);
        Logger.recordOutput(LOG_PREFIX + "AtUpperSoftLimit", isAtUpperLimit);
        Logger.recordOutput(LOG_PREFIX + "AtLowerSoftLimit", isAtLowerLimit);

        // Calculated metrics
        double power = motorVoltageSignal.getValueAsDouble() * statorCurrentSignal.getValueAsDouble();
        Logger.recordOutput(LOG_PREFIX + "PowerWatts", power);

        // Connection status
        Logger.recordOutput(LOG_PREFIX + "MotorConnected", hoodMotor.isConnected());
        Logger.recordOutput(LOG_PREFIX + "LimitSwitchConnected", limitSwitch.isConnected());
    }
}
