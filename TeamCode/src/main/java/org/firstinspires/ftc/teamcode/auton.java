package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
@Autonomous(name="auton")
public class auton extends LinearOpMode {
    DcMotor frontLeft0;
    DcMotor frontRight1;
    DcMotor backLeft2;
    DcMotor backRight3;
    DcMotor motorarm;

    IMU imu;
    final double SET_POINT = 1;
    final double kP = 0.1;
    final  double kI = 0.1;
    final  double iLimit = 0.1;
    final  double kD = 0.1;
    final double CONVERSION_RATIO = 12;//TODO This is to convert from ticks (at the motor) to feet this involves dividing by ticks/rev and by rev/feet so this should probably include ticks and wheel diameter.
    ElapsedTime time;
    double errorSum;
    double lastError;
    double lastTimeStep;
    double dt;
    @Override

    public void runOpMode() throws InterruptedException {
        time = new ElapsedTime();
        lastError = 0;
        lastTimeStep = 0;
        dt = time.seconds() - lastTimeStep;//TODO this might need to be inside the while loop
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        frontLeft0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double sensorPosition = (frontLeft0.getCurrentPosition()+frontRight1.getCurrentPosition()+backLeft2.getCurrentPosition()+backRight3.getCurrentPosition())/4*CONVERSION_RATIO;//TODO this should probably be divided by conversion ratio
            double error = SET_POINT - sensorPosition;
            if (Math.abs(error) < iLimit){
                errorSum += error * dt;
            }
            double errorRate = (error - lastError) / dt;
            double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
            frontLeft0.setPower(outputSpeed);
            frontRight1.setPower(outputSpeed);
            backLeft2.setPower(outputSpeed);
            backRight3.setPower(outputSpeed);

            lastError = error;
            telemetry.addData("Distance", sensorPosition);
            telemetry.update();



            time.reset();


        }

}}
