package frc.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class DriveWithJoystick extends Command{
    
    private Supplier<Double> m_joystickL, m_joystickR;
    private static DriveWithJoystick m_instance;
    

    public DriveWithJoystick(Supplier<Double> leftJoystick, Supplier<Double> rightJoystick){
        m_joystickL = leftJoystick;
        m_joystickR = rightJoystick;
    }

    @Override
    public void execute() {
        double leftJoystick = m_joystickL.get();
        double rightJoystick = m_joystickR.get(); 

        TankDrive.getInstance().drive(leftJoystick, rightJoystick);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
