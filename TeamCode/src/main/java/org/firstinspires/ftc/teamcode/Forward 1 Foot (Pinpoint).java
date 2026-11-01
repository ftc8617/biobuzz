java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Forward 1 Foot (Pinpoint)", group = "BioBuzz")
public class ForwardOneFoot extends LinearOpMode {

    // ---- Tuning values ----
    static final double TARGET_INCHES = 12.0;   // 1 foot
    static final double KP_DRIVE  = 0.06;       // power per inch of error
    static final double KP_STRAFE = 0.06;
    static final double KP_TURN   = 1.5;        // power per radian of error
    static final double MAX_POWER = 0.5;        // keep low for testing
    static final double MIN_POWER = 0.08;       // overcomes friction near target
    static final double TOLERANCE_IN = 0.25;
    static final double TIMEOUT_S = 3.0;

    // Pod offsets from the robot's center, in mm. MEASURE YOURS.
    // X offset: how far the forward pod is LEFT of center (right = negative)
    // Y offset: how far the strafe pod is FORWARD of center (behind = negative)
    static final double X_POD_OFFSET_MM = -84.0;
    static final double Y_POD_OFFSET_MM = -168.0;

    DcMotor frontLeft, backLeft, frontRight, backRight;
    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Typical goBILDA mecanum chassis: left side reversed
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // odometry closes the loop
        }

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();   // robot must be still during this

        // Show live odometry during init so you can check pod directions by hand
        while (opModeInInit()) {
            odo.update();
            Pose2D p = odo.getPosition();
            telemetry.addLine("Push robot forward: X should go UP");
            telemetry.addLine("Push robot left: Y should go UP");
            telemetry.addData("X (in)", "%.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", p.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        if (isStopRequested()) return;

        odo.resetPosAndIMU();   // start the run from (0, 0, 0)
        sleep(250);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TIMEOUT_S) {
            odo.update();
            Pose2D p = odo.getPosition();
            double x = p.getX(DistanceUnit.INCH);
            double y = p.getY(DistanceUnit.INCH);
            double h = p.getHeading(AngleUnit.RADIANS);

            double errX = TARGET_INCHES - x;          // forward error
            double errY = 0.0 - y;                    // drift sideways
            double errH = AngleUnit.normalizeRadians(0.0 - h);

            if (Math.abs(errX) < TOLERANCE_IN && Math.abs(errY) < 0.5) break;

            // Rotate field error into robot frame (matters if heading drifts)
            double fwdErr    =  errX * Math.cos(h) + errY * Math.sin(h);
            double strafeErr = -errX * Math.sin(h) + errY * Math.cos(h);

            double forward = Range.clip(fwdErr * KP_DRIVE, -MAX_POWER, MAX_POWER);
            if (Math.abs(forward) < MIN_POWER && Math.abs(errX) > TOLERANCE_IN)
                forward = Math.copySign(MIN_POWER, forward);
            double strafe = Range.clip(strafeErr * KP_STRAFE, -0.3, 0.3);
            double turn   = Range.clip(errH * KP_TURN, -0.3, 0.3);

            setDrive(forward, strafe, turn);

            telemetry.addData("X", "%.2f in", x);
            telemetry.addData("Y", "%.2f in", y);
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(h));
            telemetry.addData("Error X", "%.2f in", errX);
            telemetry.update();
        }

        setDrive(0, 0, 0);

        odo.update();
        Pose2D end = odo.getPosition();
        telemetry.addLine("DONE");
        telemetry.addData("Final X", "%.2f in", end.getX(DistanceUnit.INCH));
        telemetry.addData("Final Y", "%.2f in", end.getY(DistanceUnit.INCH));
        telemetry.addData("Final Heading", "%.1f deg", end.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        sleep(3000);
    }

    /** forward = +forward, strafe = +left, turn = +counter-clockwise */
    void setDrive(double forward, double strafe, double turn) {
        double fl = forward - strafe - turn;
        double bl = forward + strafe - turn;
        double fr = forward + strafe + turn;
        double br = forward - strafe + turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }
}
