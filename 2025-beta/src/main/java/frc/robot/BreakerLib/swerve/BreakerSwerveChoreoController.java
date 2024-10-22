package frc.robot.BreakerLib.swerve;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;

//TODO switch to field rel version of ApplyChassisSpeeds when support for module force FF is added
public class BreakerSwerveChoreoController implements BiConsumer<Pose2d, SwerveSample> {
    private final BreakerSwerveDrivetrain drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.ApplyChassisSpeeds request;




    public BreakerSwerveChoreoController(
        BreakerSwerveDrivetrain drivetrain,
        PIDController xController,
        PIDController yController,
        PIDController thetaController) {
        this.drivetrain = drivetrain;
        this.xController = xController;
        this.yController = yController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController = thetaController;
        request = new SwerveRequest.ApplyChassisSpeeds();
        request.DriveRequestType = DriveRequestType.Velocity;
    }

    @Override
    public void accept(Pose2d t, SwerveSample u) {
        //This section deals with the field reletive veleocitys directly commanded of the robot

        //here the "FF" is our base setpoint, idealy, this shoud be exatly what we are commanding of our drive as thease are the chassis speeds calculated by the choreo optimizer
        double xFF = u.vx;
        double yFF = u.vy;
        double rotationFF = u.omega;

        //the feedback exists to compensate for disturmbences not modeled during trajectory generation, so basicly any real workd factors that might effect path following such as varyation from the mathamatical modle of the robot, collisions, etc
        double xFeedback = xController.calculate(t.getX(), u.x);
        double yFeedback = yController.calculate(t.getY(), u.y);
        double rotationFeedback = thetaController.calculate(t.getRotation().getRadians(),
           u.heading);

        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            t.getRotation()
        );

        double[] forcesX = u.moduleForcesX();
        double[] forcesY = u.moduleForcesY();
        for (int i = 0; i < forcesX.length; i++) {
            BreakerVector2 vec = new BreakerVector2(forcesX[i], forcesY[i]);
            vec = vec.rotateBy(t.getRotation().unaryMinus());
            forcesX[i] = vec.getX();
            forcesY[i] = vec.getY();
        }

        request.Speeds = targetSpeeds;
        request.WheelForceFeedforwardsX = forcesX;
        request.WheelForceFeedforwardsY = forcesY;
        drivetrain.setControl(request);
    }
}
