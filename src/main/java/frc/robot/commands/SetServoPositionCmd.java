package frc.robot.commands;

import frc.robot.subsystems.CoralArmSubsystem;

public class SetServoPositionCmd {
    private CoralArmSubsystem m_CoralArmSubsystem;

    public void setServoPos(boolean state) {
        if (state == true) {
            m_CoralArmSubsystem.ScoreCoral();
        } else if (state == false) {
            m_CoralArmSubsystem.ResetPin();
        } else if (!state) {
            return;
        }
    }
}
