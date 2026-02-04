package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GRTUtil;

public class PDHSubsystem extends SubsystemBase {

    private final PowerDistribution pdh;

    // DataLog entries
    private final DoubleLogEntry voltageLogEntry;
    private final DoubleLogEntry totalCurrentLogEntry;
    private final DoubleLogEntry temperatureLogEntry;
    private final DoubleLogEntry totalPowerLogEntry;
    private final DoubleLogEntry totalEnergyLogEntry;
    private final DoubleLogEntry[] channelCurrentLogEntries;

    // Number of channels on REV PDH
    private static final int NUM_CHANNELS = 24;

    public PDHSubsystem(int canId) {
        pdh = new PowerDistribution(canId, ModuleType.kRev);

        // Initialize log entries
        voltageLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "pdh/voltage");
        totalCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "pdh/totalCurrent");
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "pdh/temperature");
        totalPowerLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "pdh/totalPower");
        totalEnergyLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "pdh/totalEnergy");

        // Initialize channel current log entries
        channelCurrentLogEntries = new DoubleLogEntry[NUM_CHANNELS];
        for (int i = 0; i < NUM_CHANNELS; i++) {
            channelCurrentLogEntries[i] = new DoubleLogEntry(
                DataLogManager.getLog(), "pdh/channel" + i + "/current"
            );
        }
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    public double getTemperature() {
        return pdh.getTemperature();
    }

    public double getChannelCurrent(int channel) {
        return pdh.getCurrent(channel);
    }

    public double getTotalPower() {
        return pdh.getTotalPower();
    }

    public double getTotalEnergy() {
        return pdh.getTotalEnergy();
    }

    public void resetTotalEnergy() {
        pdh.resetTotalEnergy();
    }

    private void logStats() {
        long ts = GRTUtil.getFPGATime();

        voltageLogEntry.append(getVoltage(), ts);
        totalCurrentLogEntry.append(getTotalCurrent(), ts);
        temperatureLogEntry.append(getTemperature(), ts);
        totalPowerLogEntry.append(getTotalPower(), ts);
        totalEnergyLogEntry.append(getTotalEnergy(), ts);

        // Log individual channel currents
        for (int i = 0; i < NUM_CHANNELS; i++) {
            channelCurrentLogEntries[i].append(getChannelCurrent(i), ts);
        }
    }

    private void publishStats() {
        SmartDashboard.putNumber("PDH/Voltage", getVoltage());
        SmartDashboard.putNumber("PDH/TotalCurrent", getTotalCurrent());
        SmartDashboard.putNumber("PDH/Temperature", getTemperature());
        SmartDashboard.putNumber("PDH/TotalPower", getTotalPower());
    }

    @Override
    public void periodic() {
        logStats();
        publishStats();
    }
}
