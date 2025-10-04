package org.firstinspires.ftc.teamcode.test;

import org.junit.jupiter.api.AssertionFailureBuilder;
import org.junit.jupiter.api.function.Executable;
import static org.junit.jupiter.api.Assertions.assertTrue;


public class TestUtil {
    /**
     * Returns true if the given code throws an exception; otherwise, it
     * returns false. This is useful for assumptions dependent on other 
     * tests.
     * 
     * @param code What to execute to test for thrown exceptions.
     */
    public static boolean doesThrow(Executable code) {
        try {
            code.execute();
            return false;
        } catch(Throwable err) {
            return true;
        } 
    }

    /**
     * Asserts that the actual value is within tolerance of the expected value.
     * This is useful for floating point operations that can accumulate error
     * or operations that themselves rely on tolerance.
     * 
     * This method is an alias of `assertWithin()`
     * 
     * @param expected The value that some operation should get
     * @param actual Output of the operation
     * @param eps The tolerance, above or below. Tolerance is inclusive
     */
    public static void assertFloatEquals(double expected, double actual, double eps){
        assertWithin(expected, actual, eps);
    }

    /**
     * Asserts that the actual value is within tolerance of the expected value.
     * This is useful for floating point operations that can accumulate error
     * or operations that themselves rely on tolerance.
     * 
     * @param expected The value that some operation should get
     * @param actual Output of the operation
     * @param eps The tolerance, above or below. Is exclusive (equality fails)
     */
    public static void assertWithin(double expected, double actual, double eps) {
        if(Math.abs(expected - actual) > eps) {
            // throw ("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">");
            // throw new RuntimeException("waaat? o_O");
            AssertionFailureBuilder
                .assertionFailure()
                .reason("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                // .message("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                // .actual(actual)
                // .expected(expected)
                .buildAndThrow();
        }
    }

    /**
     * Asserts that the actual value is not within tolerance of the expected 
     * value. This is useful for floating point operations that can accumulate 
     * error or operations that themselves rely on tolerance, but that you
     * want to verify something changed.
     * 
     * This method is an alias of `assertNotWithin()`
     * 
     * @param unexpected The value that some operation should not get
     * @param actual Output of the operation
     * @param eps The tolerance, above or below. Is inclusive (equality fails)
     */
    public static void assertFloatNotEquals(double unexpected, double actual, double eps) {
        assertNotWithin(unexpected, actual, eps);
    }
    

    /**
     * Asserts that the actual value is not within tolerance of the expected 
     * value. This is useful for floating point operations that can accumulate 
     * error or operations that themselves rely on tolerance, but that you
     * want to verify something changed.
     * 
     * @param unexpected The value that some operation should not get
     * @param actual Output of the operation
     * @param eps The tolerance, above or below. Is inclusive (equality fails)
     */
    public static void assertNotWithin(double unexpected, double actual, double eps) {
        if(Math.abs(unexpected - actual) < eps) {
            // throw ("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">");
            // throw new RuntimeException("waaat? o_O");
            AssertionFailureBuilder
                .assertionFailure()
                .reason("Did not expect: <" + unexpected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                // .message("expected: <" + expected + "> with tolerance: <" + eps + "> but was: <" + actual + ">")
                // .actual(actual)
                // .expected(expected)
                .buildAndThrow();
        }
    }
}