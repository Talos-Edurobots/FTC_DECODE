package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.GenericValue;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.Map;

@Configurable
@TeleOp(name = "PanelsConfigurablesTest", group = "testing")
public class PanelsConfigurablesTest extends LinearOpMode {
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static int testInt = 1;
    public static long testLong = 1L;
    public static double testDouble = 1.0;
    public static float testFloat = 1.0f;
    public static String testString = "test!";
    public static boolean testBoolean = false;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.aWasPressed()) {
                testBoolean ^= true;
            }
            telemetryM.addData("testInt", testInt);
            telemetryM.addData("testLong", testLong);
            telemetryM.addData("testDouble", testDouble);
            telemetryM.addData("testFloat", testFloat);
            telemetryM.addData("testString", testString);
            telemetryM.addData("testBoolean", testBoolean);
            telemetryM.addData("testEnum", testEnum);
            telemetryM.addData("testArray", testArray);
            telemetryM.addData("testList", testList);
            telemetryM.addData("testMap", testMap);
            telemetryM.addData("testCustomType", testCustomType);
            telemetryM.addData("testNestedType", testNestedType);
            telemetryM.addData("testUnknownType", testUnknownType);
            telemetryM.addData("testRandomArray", testRandomArray);
            telemetryM.addData("testTParamClass", testTParamClass);
            telemetryM.update(telemetry);
        }
    }

    public enum TestEnum {
        TEST1,
        TEST2,
        TEST3
    }

    public static TestEnum testEnum = TestEnum.TEST1;
    public static int[] testArray = {1, 2, 3};
    public static List<Integer> testList = List.of(1, 2, 3);
    public static Map<String, Integer> testMap = Map.of("one", 1, "two", 2, "three", 3);

    public static class CustomType {
        public final int testInt;
        public final String testString;

        public CustomType(int testInt, String testString) {
            this.testInt = testInt;
            this.testString = testString;
        }

        public int getTestInt() {
            return testInt;
        }

        public String getTestString() {
            return testString;
        }
    }

    public static CustomType testCustomType = new CustomType(1, "test!");

    public static class NestedType {
        public final int testInt;
        public final String testString;
        public final CustomType testCustomType;

        public NestedType(int testInt, String testString, CustomType testCustomType) {
            this.testInt = testInt;
            this.testString = testString;
            this.testCustomType = testCustomType;
        }

        public int getTestInt() {
            return testInt;
        }

        public String getTestString() {
            return testString;
        }

        public CustomType getTestCustomType() {
            return testCustomType;
        }
    }

    public static NestedType testNestedType = new NestedType(1, "test!", new CustomType(2, "test2!"));

    public static class UnknownType {
        public final int testInt;

        public UnknownType(int testInt) {
            this.testInt = testInt;
        }

        public int getTestInt() {
            return testInt;
        }
    }

    public static UnknownType testUnknownType = new UnknownType(1);

    public static Object[] testRandomArray = new Object[]{
            1,
            1.0,
            1.0f,
            "test!",
            true,
            new CustomType(1, "test!"),
            new NestedType(1, "test!", new CustomType(2, "test2!")),
            new UnknownType(1),
            new int[]{1, 2, 3},
            Map.of("one", 1, "two", 2, "three", 3)
    };

    public static class TParamClass<T> {
        public final T test;

        public TParamClass(T test) {
            this.test = test;
        }

        public T getTest() {
            return test;
        }
    }

    @GenericValue(tParam = Integer.class)
    public static TParamClass<Integer> testTParamClass = new TParamClass<Integer>(1);
}