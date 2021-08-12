package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveTrain extends SubsystemBase{
    
    private final TalonSRX FLdrive = new TalonSRX(0);
    private final TalonSRX FRdrive = new TalonSRX (1);
    private final TalonSRX BLdrive = new TalonSRX (2);
    private final TalonSRX BRdrive = new TalonSRX (3);

    public void arcadeDrive(double speed, double turn) {

        speed = speed * Math.abs(speed);
        turn = turn * Math.abs(turn);

        double left = speed + turn;
        double right = speed - turn;

        setPower(left, right);
    }
    public void setPower(double left, double right) {

        FLdrive.set(ControlMode.PercentOutput, left);
        BLdrive.set(ControlMode.PercentOutput, left);
        FRdrive.set(ControlMode.PercentOutput, -right);
        BRdrive.set(ControlMode.PercentOutput, -right);
    
    }
}
