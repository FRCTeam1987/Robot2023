package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import org.opencv.core.RotatedRect;

import java.util.Arrays;

public class ArmIOTalonFX implements ArmIO {
    private TalonFX rotationLeader;
    private TalonFX rotationFollower;
    private CANCoder rotationEncoder;

    private TalonFX telescopingMotor;


    static double heightOffset = 12.0;

    public ArmIOTalonFX(int leaderMotorID, int followerMotorID) {
        rotationLeader = new TalonFX(leaderMotorID);
        rotationFollower = new TalonFX(followerMotorID);
        rotationFollower.follow(rotationLeader);
        //rotationEncoder = new CANCoder(rotationCANCoderID);

        //telescopingMotor = new TalonFX(telescopingMotorID);


    }



    public static double calculateArmLength(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow((y-heightOffset), 2));
    }

    public static double[] calculateArmPositionXy(double angle, double armLength) {
        double angleRad = Math.toRadians(angle);
        double x = armLength * Math.cos(angleRad);
        double y = armLength * Math.sin(angleRad) + heightOffset;
        return new double[]{x,y};
    }

    public static double calculateArmAngle(double x, double y) {
        if (x == 0 && y > heightOffset) return 90.0;
        if (x == 0 && y < heightOffset) System.out.println("Impossible!"); //DriverStation.reportError("Impossible angle reached!", true);
        if (x>0) return Math.toDegrees(Math.atan((y-heightOffset)/x));
        if (x<0) return 180 + Math.toDegrees(Math.atan((y-heightOffset)/x));
        return 0.0;
    }

    @Override
    public synchronized void updateInputs(ArmIOInputs inputs) {
        double[] amps = new double[2];
        amps[0] = rotationLeader.getStatorCurrent();
        amps[1] = rotationFollower.getStatorCurrent();
        inputs.currentAmps = amps;
    }
}
