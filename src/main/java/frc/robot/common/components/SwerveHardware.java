package frc.robot.common.components;


import frc.robot.common.swerve.RAWRNavX2;
import frc.robot.common.swerve.RAWRSwerveModule;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class SwerveHardware {
    public final RAWRNavX2 navx;
    public final RAWRSwerveModule lFrontModule;
    public final RAWRSwerveModule rFrontModule;
    public final RAWRSwerveModule lRearModule;
    public final RAWRSwerveModule rRearModule;


    public void lock() {
        lFrontModule.lock();
        rFrontModule.lock();
        lRearModule.lock();
        rRearModule.lock();
    }

    public void stop() {
        lFrontModule.stop();
        rFrontModule.stop();
        lRearModule.stop();
        rRearModule.stop();
    }

    public void toggleTractionControl() {
        lFrontModule.toggleTractionControl();
        rFrontModule.toggleTractionControl();
        lRearModule.toggleTractionControl();
        rRearModule.toggleTractionControl();
    }

    public void enableTractionControl() {
        lFrontModule.enableTractionControl();
        rFrontModule.enableTractionControl();
        lRearModule.enableTractionControl();
        rRearModule.enableTractionControl();
    }

    public void disableTractionControl() {
        lFrontModule.disableTractionControl();
        rFrontModule.disableTractionControl();
        lRearModule.disableTractionControl();
        rRearModule.disableTractionControl();
    }
}