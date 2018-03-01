package org.usfirst.frc.team3310.lib.util.drivers;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;
import org.usfirst.frc.team3310.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

@RunWith(PowerMockRunner.class)
@PrepareForTest(TalonSRXFactory.class)
public class TalonSRXFactoryTest {

    @Test
    public void testWhichMethodsAreCalled() throws Exception {
        List<String> acceptableUncalledMethodNames = Arrays.asList(
                "getClass",
                "pidWrite",
                "wait",
                "notifyAll",
                "delete",
                "disableControl",
                "notify",
                "setF",
                "setI",
                "hashCode",
                "enable",
                "setD",
                "setParameter",
                "setP",
                "setPIDSourceType",
                "initTable",
                "getParameter",
                "getMotionProfileStatus",
                "startLiveWindowMode",
                "enableControl",
                "updateTable",
                "setSetpoint",
                "GetGadgeteerStatus",
                "disable",
                "pushMotionProfileTrajectory",
                "equals",
                "reset",
                "stopMotor",
                "processMotionProfileBuffer",
                "setControlMode",
                "stopLiveWindowMode",
                "getMotionMagicActTrajPosition",
                "getMotionMagicActTrajVelocity",
                "DisableNominalClosedLoopVoltage",
                "createTableListener");

        final Set<String> uncalledMethodNames = new HashSet<>(
                Arrays.stream(TalonSRX.class.getMethods())
                        .map(m -> m.getName())
                        .filter(name -> !acceptableUncalledMethodNames.contains(name))
                        .collect(Collectors.toSet()));

        TalonSRX talon = Mockito.mock(TalonSRX.class, new Answer() {
            @Override
            public Object answer(InvocationOnMock invocationOnMock) throws Throwable {
                uncalledMethodNames.remove(invocationOnMock.getMethod().getName());
                return null;
            }
        });
        PowerMockito.whenNew(TalonSRX.class).withAnyArguments().thenReturn(talon);

        TalonSRX returnedTalon = TalonSRXFactory.createDefaultTalon(1);
        String talonInfo = TalonSRXFactory.getFullTalonInfo(returnedTalon);

        Assert.assertEquals(
                new HashSet<>(),
                uncalledMethodNames);
    }

    @Test
    public void testCanPrintInfo() {
        System.out.println(TalonSRXFactory.getFullTalonInfo(Mockito.mock(TalonSRX.class)));
    }
}
