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

/*
 * Mecanum autonomous using 2 goBILDA 4-Bar Odometry Pods on a goBILDA Pinpoint.
 *   1) Drive forward 12 in
 *   2) Wait 1 second
 *   3) Drive to the LOADING ZONE (PARK = 5 AUTO points in BIOBUZZ)
 *
 * Pinpoint coordinates (from the start position):
 *   +X = forward, +Y = left, +heading = counter-clockwise
 */
@Autonomous(name = "Mecanum Odo: Fwd 1ft -> Loading Zone", group = "Auto")
public class MecanumOdoAuto extends LinearOpMode {

    // ---------------- ALLIANCE ----------------
    static final boolean RED_ALLIANCE = true;   // set false for blue (mirrors Y)

    // ------------- TARGETS (inches, measured from robot start) -------------
    static final double FWD_X = 12.0;            // step 1: forward 1 ft
    static final double FWD_Y = 0.0;

    static final double LOAD_X = 12.0;           // step 3: LOADING ZONE -- MEASURE ON YOUR FIELD
    static final double LOAD_Y = -36.0;          //   (negative Y = to the robot's right)
    static final double LOAD_HEADING_DEG = 0.0;  // keep facing the same way

    // ------------- PINPOINT SETUP (measure your robot) -------------
    static final double X_POD_OFFSET_MM = -84.0;   // X pod: sideways from center, left = +
    static final double Y_POD_OFFSET_MM = -168.0;  // Y pod: fore/aft from center, forward = +

    // ------------- CONTROL TUNING -------------
    static final double MAX_POWER     = 0.5;
    static final double MIN_POWER     = 0.12;   // enough to overcome friction
    static final double KP_DRIVE      = 0.05;   // power per inch of error
    static final double KP_TURN       = 1.0;    // power per radian of heading error
    static final double POS_TOL_IN    = 0.75;
    static final double HEAD_TOL_DEG  = 2.0;
    static final double SETTLE_S      = 0.15;   // must stay in tolerance this long

    DcMotor leftFront, leftBack, rightFront, rightBack;
    GoBildaPinpointDriver pinpoint;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---- Drive motors (same names/directions as your mecanum encoder code) ----
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // pods do the measuring
        }

        // ---- Pinpoint ----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();   // robot must be still

        // Show live position during INIT so you can check the pods
        while (opModeInInit()) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addData("X (in)", "%.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", p.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Alliance", RED_ALLIANCE ? "RED" : "BLUE");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Start counts as (0,0,0) even if the robot was bumped during INIT
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        double ySign = RED_ALLIANCE ? 1.0 : -1.0;

        // 1) Forward 1 foot
        driveTo(FWD_X, FWD_Y * ySign, 0.0, 4.0);

        // 2) Wait 1 second
        stopMotors();
        sleep(1000);

        // 3) Drive to the LOADING ZONE
        driveTo(LOAD_X, LOAD_Y * ySign, LOAD_HEADING_DEG * ySign, 6.0);

        stopMotors();
        telemetry.addData("Path", "Complete - parked");
        telemetry.update();
        while (opModeIsActive()) { idle(); }   // hold until AUTO ends
    }

    /** Drive to a field position (inches) and heading (degrees) using Pinpoint odometry. */
    void driveTo(double targetX, double targetY, double targetHeadingDeg, double timeoutS) {

        double targetH = Math.toRadians(targetHeadingDeg);
        ElapsedTime settle = new ElapsedTime();
        boolean inTol = false;
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutS) {

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double h = pose.getHeading(AngleUnit.RADIANS);

            // Error in field frame
            double dx = targetX - x;
            double dy = targetY - y;
            double dist = Math.hypot(dx, dy);
            double hErr = AngleUnit.normalizeRadians(targetH - h);

            // Done?
            if (dist < POS_TOL_IN && Math.abs(Math.toDegrees(hErr)) < HEAD_TOL_DEG) {
                if (!inTol) { inTol = true; settle.reset(); }
                if (settle.seconds() > SETTLE_S) break;
            } else {
                inTol = false;
            }

            // Rotate field error into robot frame
            double forward =  dx * Math.cos(h) + dy * Math.sin(h);
            double left    = -dx * Math.sin(h) + dy * Math.cos(h);

            // Proportional speed toward target, with min/max limits
            double speed = 0;
            if (dist > POS_TOL_IN) {
                speed = Range.clip(KP_DRIVE * dist, MIN_POWER, MAX_POWER);
            }
            double fwdPow  = (dist > 0) ? forward / dist * speed : 0;
            double leftPow = (dist > 0) ? left    / dist * speed : 0;
            double turnPow = Range.clip(KP_TURN * hErr, -MAX_POWER, MAX_POWER);

            setMecanum(fwdPow, leftPow, turnPow);

            telemetry.addData("Target", "X %.1f  Y %.1f  H %.1f", targetX, targetY, targetHeadingDeg);
            telemetry.addData("Pose",   "X %.1f  Y %.1f  H %.1f", x, y, Math.toDegrees(h));
            telemetry.addData("Error",  "dist %.2f in  head %.1f deg", dist, Math.toDegrees(hErr));
            telemetry.update();
        }
        stopMotors();
    }

    /** forward +, left +, turn + = counter-clockwise */
    void setMecanum(double forward, double left, double turn) {
        double lf = forward - left - turn;
        double lb = forward + left - turn;
        double rf = forward + left + turn;
        double rb = forward - left + turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                                            Math.max(Math.abs(rf), Math.abs(rb))));
        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    void stopMotors() {
        setMecanum(0, 0, 0);
    }
}
