// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

// import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import first.robot.utilities.HubShiftUtil;

public class DataLogger extends SubsystemBase {
    // private final PowerDistribution m_powerDist = new PowerDistribution();

    public DataLogger() {
    }

    @Override
    public void periodic() {
        // Power
        // SmartDashboard.putNumber("power/totalCurrent", m_powerDist.getTotalCurrent());
        SmartDashboard.putNumber("power/batteryVoltage", RobotController.getBatteryVoltage());

        // Network
        SmartDashboard.putNumber("network/CAN Bus Utilization",
                RobotBase.isSimulation() ? 0.0 : RobotController.getCANStatus(0).percentBusUtilization);

        HubShiftUtil.ShiftInfo officialShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        HubShiftUtil.ShiftInfo shiftedShiftInfo = HubShiftUtil.getShiftedShiftInfo(HubShiftUtil.getProjectileLeadTimeSec());
        boolean hubTimingRelevant = HubShiftUtil.isHubTimingRelevant();
        boolean hubActiveNow = officialShiftInfo.active();
        boolean clearToShoot = hubTimingRelevant && shiftedShiftInfo.active();

        // SmartDashboard
        SmartDashboard.putNumber("shoot/matchElapsedSec", HubShiftUtil.getMatchElapsedSec());
        SmartDashboard.putNumber("shoot/matchRemainingSec", HubShiftUtil.getMatchRemainingSec());
        SmartDashboard.putNumber("shoot/hubShiftElapsedSec", shiftedShiftInfo.elapsedTimeSec());
        SmartDashboard.putNumber("shoot/hubShiftRemainingSec", shiftedShiftInfo.remainingTimeSec());
        SmartDashboard.putBoolean("shoot/hubActiveNow", hubActiveNow);
        SmartDashboard.putBoolean("shoot/clearToShoot", clearToShoot);
        SmartDashboard.putBoolean("shoot/hubTimingRelevant", hubTimingRelevant);

        // RobotLog
        SmartDashboard.putString("shoot/hubShiftState", officialShiftInfo.currentShift().name());
        SmartDashboard.putString("shoot/shiftedHubShiftState", shiftedShiftInfo.currentShift().name());
        SmartDashboard.putNumber("shoot/projectileLeadTimeSec", HubShiftUtil.getProjectileLeadTimeSec());
    }
}
