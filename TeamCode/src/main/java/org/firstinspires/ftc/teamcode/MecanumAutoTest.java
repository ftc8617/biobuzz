package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Forward 1 ft", group = "Auto")
public class MecanumAutoTest extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    ElapsedTime runtime = new ElapsedTime();

    // ---- SET THESE FOR YOUR HARDWARE ----
    static final double COUNTS_PER_MOTOR_REV  = 537.7;  // goBILDA 5203 312 RPM
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09;   // goBILDA 104 mm mecanum
    // -------------------------------------

    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double STRAFE_FACTOR = 1.1;  // mecanum strafing slips; tune if you strafe
    static final double DRIVE_SPEED   = 0.4;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        leftBack   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightDrive"); 
        rightBack  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Typical goBILDA mecanum: left side reversed. Verify with the wheel test below.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Counts per inch", "%.1f", COUNTS_PER_INCH);
        telemetry.addData("Encoders", "LF:%d LB:%d RF:%d RB:%d",
                leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
        telemetry.update();

        waitForStart();

        // Forward 12 inches, no strafe, 5 second timeout
        mecanumDrive(DRIVE_SPEED, 36.0, 0.0, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * forwardInches: + forward / - backward
     * strafeInches:  + right   / - left
     */
    public void mecanumDrive(double speed, double forwardInches, double strafeInches, double timeoutS) {

        if (!opModeIsActive()) return;

        int fwd    = (int) Math.round(forwardInches * COUNTS_PER_INCH);
        int strafe = (int) Math.round(strafeInches  * COUNTS_PER_INCH * STRAFE_FACTOR);

        int lfTarget = leftFront.getCurrentPosition()  + fwd + strafe;
        int lbTarget = leftBack.getCurrentPosition()   + fwd - strafe;
        int rfTarget = rightFront.getCurrentPosition() + fwd - strafe;
        int rbTarget = rightBack.getCurrentPosition()  + fwd + strafe;

        // Target MUST be set before switching to RUN_TO_POSITION
        leftFront.setTargetPosition(lfTarget);
        leftBack.setTargetPosition(lbTarget);
        rightFront.setTargetPosition(rfTarget);
        rightBack.setTargetPosition(rbTarget);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        runtime.reset();
        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setPower(Math.abs(speed));
        }

        while (opModeIsActive()
                && runtime.seconds() < timeoutS
                && leftFront.isBusy() && leftBack.isBusy()
                && rightFront.isBusy() && rightBack.isBusy()) {

            telemetry.addData("Target",  "LF:%d LB:%d RF:%d RB:%d", lfTarget, lbTarget, rfTarget, rbTarget);
            telemetry.addData("Current", "LF:%d LB:%d RF:%d RB:%d",
                    leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }
}
