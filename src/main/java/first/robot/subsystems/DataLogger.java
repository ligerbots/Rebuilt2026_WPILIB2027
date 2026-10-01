// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
// import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;

import first.robot.utilities.HubShiftUtil;

public class DataLogger extends SubsystemBase {
    // private final PowerDistribution m_powerDist = new PowerDistribution();

    public DataLogger() {
    }

    @Override
    public void periodic() {
        // Power
        // Telemetry.log("power/totalCurrent", m_powerDist.getTotalCurrent());
        Telemetry.log("power/batteryVoltage", RobotController.getBatteryVoltage());

        // Network
        Telemetry.log("network/CAN0 Bus Utilization",
                RobotBase.isSimulation() ? 0.0 : RobotController.getCANStatus(CANPort.CAN_D0).percentBusUtilization);

        HubShiftUtil.ShiftInfo officialShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        HubShiftUtil.ShiftInfo shiftedShiftInfo = HubShiftUtil.getShiftedShiftInfo(HubShiftUtil.getProjectileLeadTimeSec());
        boolean hubTimingRelevant = HubShiftUtil.isHubTimingRelevant();
        boolean hubActiveNow = officialShiftInfo.active();
        boolean clearToShoot = hubTimingRelevant && shiftedShiftInfo.active();

        // SmartDashboard
        Telemetry.log("shoot/matchElapsedSec", HubShiftUtil.getMatchElapsedSec());
        Telemetry.log("shoot/matchRemainingSec", HubShiftUtil.getMatchRemainingSec());
        Telemetry.log("shoot/hubShiftElapsedSec", shiftedShiftInfo.elapsedTimeSec());
        Telemetry.log("shoot/hubShiftRemainingSec", shiftedShiftInfo.remainingTimeSec());
        Telemetry.log("shoot/hubActiveNow", hubActiveNow);
        Telemetry.log("shoot/clearToShoot", clearToShoot);
        Telemetry.log("shoot/hubTimingRelevant", hubTimingRelevant);

        // RobotLog
        Telemetry.log("shoot/hubShiftState", officialShiftInfo.currentShift().name());
        Telemetry.log("shoot/shiftedHubShiftState", shiftedShiftInfo.currentShift().name());
        Telemetry.log("shoot/projectileLeadTimeSec", HubShiftUtil.getProjectileLeadTimeSec());
    }
}
