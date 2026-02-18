package frc.robot.util;

import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedTalonFX extends TalonFX {
    enum TelemetryLevel {
        BASIC,
        STANDARD,
        DETAILED,
        FULL;

        boolean includes(TelemetryLevel required) {
            return this.ordinal() >= required.ordinal();
        }
    }

    private static final TelemetryLevel DEFAULT_TELEMETRY_LEVEL = TelemetryLevel.BASIC;
    private static final double DEFAULT_UPDATE_HZ = 10.0;

    private final String dashboardKey; 
    private final String logPrefix;

    public TelemetryLevel dashboardTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;
    public TelemetryLevel logTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;

    public double updatesPerSec = 1.0 / DEFAULT_UPDATE_HZ;
    private double lastUpdateTime = 0;

    // ===== Cached signals (refresh once, then read many) =====
    private final StatusSignal<Angle> position = getPosition();
    private final StatusSignal<AngularVelocity> velocity = getVelocity();
    private final StatusSignal<AngularAcceleration> acceleration = getAcceleration();

    private final StatusSignal<Double> dutyCycle = getDutyCycle();
    private final StatusSignal<Voltage> supplyVoltage = getSupplyVoltage();
    private final StatusSignal<Current> supplyCurrent = getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = getStatorCurrent();

    private final StatusSignal<Temperature> deviceTemp = getDeviceTemp();

    private final StatusSignal<Current> torqueCurrent = getTorqueCurrent();
    private final StatusSignal<Per<TorqueUnit, CurrentUnit>> motorKt = getMotorKT();

    private final StatusSignal<Double> closedLoopReference = getClosedLoopReference();
    private final StatusSignal<Double> closedLoopError = getClosedLoopError();

    private final StatusSignal<ForwardLimitValue> forwardLimit = getForwardLimit();
    private final StatusSignal<ReverseLimitValue> reverseLimit = getReverseLimit();

    private final StatusSignal<Boolean> faultBridgeBrownout = getFault_BridgeBrownout();
    private final StatusSignal<Boolean> faultHardware = getFault_Hardware();
    private final StatusSignal<Boolean> faultBootDuringEnable = getFault_BootDuringEnable();


    public LoggedTalonFX(int deviceId, String canBus) {
        this(deviceId, canBus, ("motor" + deviceId));
    }

    public LoggedTalonFX(int deviceId, String canBus, String dashboardKey) {
        super(deviceId, canBus);
        this.logPrefix = "/TalonFX/" + dashboardKey.replace("/", "_").replace(" ", "_");
        this.dashboardKey = dashboardKey;
    }

    public void setupDashboard() {
        SmartDashboard.putNumber(dashboardKey + "/Position", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/Velocity", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/Acceleration", 0.0);

        SmartDashboard.putNumber(dashboardKey + "/DutyCycle", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/SupplyVoltage", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/SupplyCurrent", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/StatorCurrent", 0.0);

        SmartDashboard.putNumber(dashboardKey + "/TempC", 0.0);

        SmartDashboard.putNumber(dashboardKey + "/TorqueCurrent", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/TorqueNm", 0.0);

        SmartDashboard.putNumber(dashboardKey + "/ClosedLoopRef", 0.0);
        SmartDashboard.putNumber(dashboardKey + "/ClosedLoopError", 0.0);

        SmartDashboard.putBoolean(dashboardKey + "/ForwardLimit", false);
        SmartDashboard.putBoolean(dashboardKey + "/ReverseLimit", false);

        SmartDashboard.putBoolean(dashboardKey + "/Fault/Brownout", false);
        SmartDashboard.putBoolean(dashboardKey + "/Fault/Hardware", false);
        SmartDashboard.putBoolean(dashboardKey + "/Fault/BootDuringEnable", false);
    }

    /** Call once in robotInit after DataLogManager.start(). */
    public void setupLog(DataLog log) {
        if (log == null) {
            logReady = false;
            return;
        }

        posLog = new DoubleLogEntry(log, logPrefix + "/Position");
        velLog = new DoubleLogEntry(log, logPrefix + "/Velocity");
        accelLog = new DoubleLogEntry(log, logPrefix + "/Acceleration");

        dutyLog = new DoubleLogEntry(log, logPrefix + "/DutyCycle");
        voltsLog = new DoubleLogEntry(log, logPrefix + "/SupplyVoltage");
        supCurLog = new DoubleLogEntry(log, logPrefix + "/SupplyCurrent");
        statCurLog = new DoubleLogEntry(log, logPrefix + "/StatorCurrent");

        tempLog = new DoubleLogEntry(log, logPrefix + "/TempC");

        tqCurLog = new DoubleLogEntry(log, logPrefix + "/TorqueCurrent");
        tqNmLog = new DoubleLogEntry(log, logPrefix + "/TorqueNm");

        clRefLog = new DoubleLogEntry(log, logPrefix + "/ClosedLoopRef");
        clErrLog = new DoubleLogEntry(log, logPrefix + "/ClosedLoopError");

        fwdLimitLog = new BooleanLogEntry(log, logPrefix + "/ForwardLimit");
        revLimitLog = new BooleanLogEntry(log, logPrefix + "/ReverseLimit");

        brownoutLog = new BooleanLogEntry(log, logPrefix + "/Fault/Brownout");
        hardwareLog = new BooleanLogEntry(log, logPrefix + "/Fault/Hardware");
        bootEnableLog = new BooleanLogEntry(log, logPrefix + "/Fault/BootDuringEnable");

        logReady = true;
    }

    /**
     * Refresh all cached signals from CAN. This is invoked automatically whenever
     * a dashboard or log update is performed.
     *
     * @return Status of the CAN refresh transaction.
     */
    private StatusCode refreshSignals() {
        return BaseStatusSignal.refreshAll(
                position,
                velocity,
                acceleration,
                dutyCycle,
                supplyVoltage,
                supplyCurrent,
                statorCurrent,
                deviceTemp,
                torqueCurrent,
                motorKt,
                closedLoopReference,
                closedLoopError,
                forwardLimit,
                reverseLimit,
                faultBridgeBrownout,
                faultHardware,
                faultBootDuringEnable);
    }

    /**
     * Publish the latest signal values to SmartDashboard (rate limited by
     * configuration).
     */
    public void updateDashboard() {
        if (!shouldRunDashboardUpdate()) {
            return;
        }
        refreshSignals();

        if (shouldIncludeOnDashboard(TelemetryLevel.BASIC)) {
            SmartDashboard.putNumber(dashboardKey + "/Position", position.getValue().in(Units.Radians));
            SmartDashboard.putNumber(dashboardKey + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond));
            SmartDashboard.putNumber(dashboardKey + "/DutyCycle", dutyCycle.getValue());

            SmartDashboard.putBoolean(dashboardKey + "/ForwardLimit", isForwardLimitClosed());
            SmartDashboard.putBoolean(dashboardKey + "/ReverseLimit", isReverseLimitClosed());

            SmartDashboard.putBoolean(dashboardKey + "/Fault/Brownout", faultBridgeBrownout.getValue());
            SmartDashboard.putBoolean(dashboardKey + "/Fault/Hardware", faultHardware.getValue());
            SmartDashboard.putBoolean(dashboardKey + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue());
        }

        if (shouldIncludeOnDashboard(TelemetryLevel.STANDARD)) {
            SmartDashboard.putNumber(
                    dashboardKey + "/Acceleration", acceleration.getValue().in(Units.RadiansPerSecondPerSecond));
            SmartDashboard.putNumber(dashboardKey + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts));
            SmartDashboard.putNumber(dashboardKey + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps));
        }

        if (shouldIncludeOnDashboard(TelemetryLevel.DETAILED)) {
            SmartDashboard.putNumber(dashboardKey + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps));
            SmartDashboard.putNumber(dashboardKey + "/TempC", deviceTemp.getValue().in(Units.Celsius));
        }

        if (shouldIncludeOnDashboard(TelemetryLevel.FULL)) {
            SmartDashboard.putNumber(dashboardKey + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps));
            SmartDashboard.putNumber(dashboardKey + "/TorqueNm", getTorque().in(NewtonMeters));

            SmartDashboard.putNumber(dashboardKey + "/ClosedLoopRef", closedLoopReference.getValue());
            SmartDashboard.putNumber(dashboardKey + "/ClosedLoopError", closedLoopError.getValue());
        }
    }

    /**
     * Append the latest signal values to WPILib DataLog (rate limited by
     * configuration).
     */
    public void updateLog() {
        if (!logReady || !shouldRunLogUpdate()) {
            return;
        }

        refreshSignals();
        if (shouldIncludeInLog(TelemetryLevel.BASIC)) {
            posLog.append(position.getValue().in(Units.Radians));
            velLog.append(velocity.getValue().in(Units.RadiansPerSecond));
            dutyLog.append(dutyCycle.getValue());

            fwdLimitLog.append(isForwardLimitClosed());
            revLimitLog.append(isReverseLimitClosed());

            brownoutLog.append(faultBridgeBrownout.getValue());
            hardwareLog.append(faultHardware.getValue());
            bootEnableLog.append(faultBootDuringEnable.getValue());
        }

        if (shouldIncludeInLog(TelemetryLevel.STANDARD)) {
            accelLog.append(acceleration.getValue().in(Units.RadiansPerSecondPerSecond));
            voltsLog.append(supplyVoltage.getValue().in(Units.Volts));
            supCurLog.append(supplyCurrent.getValue().in(Units.Amps));
        }

        if (shouldIncludeInLog(TelemetryLevel.DETAILED)) {
            statCurLog.append(statorCurrent.getValue().in(Units.Amps));
            tempLog.append(deviceTemp.getValue().in(Units.Celsius));
        }

        if (shouldIncludeInLog(TelemetryLevel.FULL)) {
            tqCurLog.append(torqueCurrent.getValue().in(Units.Amps));
            tqNmLog.append(getTorque().in(NewtonMeters));

            clRefLog.append(closedLoopReference.getValue());
            clErrLog.append(closedLoopError.getValue());
        }
    }

    private boolean shouldIncludeOnDashboard(TelemetryLevel required) {
        return dashboardTelemetryLevel.includes(required);
    }

    private boolean shouldIncludeInLog(TelemetryLevel required) {
        return logTelemetryLevel.includes(required);
    }

    private boolean shouldRunDashboardUpdate() {
        double now = Timer.getFPGATimestamp();
        if (now - lastDashboardUpdateTime < dashboardUpdatePerSec) {
            return false;
        }
        lastDashboardUpdateTime = now;
        return true;
    }

    private boolean shouldRunLogUpdate() {
        double now = Timer.getFPGATimestamp();
        if (now - lastLogUpdateTime < logUpdatePerSec) {
            return false;
        }
        lastLogUpdateTime = now;
        return true;
    }

    private boolean isForwardLimitClosed() {
        return forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }

    private boolean isReverseLimitClosed() {
        return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
    }

    private Torque getTorque() {
        Per<TorqueUnit, CurrentUnit> kt = motorKt.getValue();
        Current tCurrent = torqueCurrent.getValue();
        return (Torque) kt.timesDivisor(tCurrent);
    }
}