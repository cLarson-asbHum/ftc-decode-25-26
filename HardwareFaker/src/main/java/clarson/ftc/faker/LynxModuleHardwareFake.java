package clarson.ftc.faker;

import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Unfortunately, we cannot call this this "LynxModuleHardwareFake" because either Gradle 
// or Java ignores that the file exists when it has the name. We instead opt with the
// next best option, "LynxModuleHardwareFake". This is horribly inconsisten with other 
// fakes, and pains me to write.
//
// This bug has cost me 30 minutes and several coins to the swear jar.
public class LynxModuleHardwareFake extends LynxModule {
    public LynxModuleHardwareFake(LynxUsbDeviceImplFake usbDevice, int moduleAddress, boolean isParent, boolean isUser) {
        super(usbDevice, moduleAddress, isParent, isUser);
    }

    /**
     * This method "recordBulkCachingCommandIntent", modified from the original source form,
     * is subject to the following:
     * 
     * Copyright (c) 2016 Robert Atkinson
     * 
     * All rights reserved.
     * 
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     * 
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     * 
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     * 
     * Neither the name of Robert Atkinson nor the names of his contributors may be used to
     * endorse or promote products derived from this software without specific prior
     * written permission.
     * 
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
     * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
     * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */
    public BulkData recordBulkCachingCommandIntent(LynxDekaInterfaceCommand<?> command, String tag) {
        warnIfClosed();
        synchronized (bulkCachingLock) {
            List<LynxDekaInterfaceCommand<?>> commands = bulkCachingHistory.get(tag);

            if (bulkCachingMode == BulkCachingMode.AUTO) {
                // automatically clear the cache if necessary based on the command history
                if (commands == null) {
                    commands = new ArrayList<>();
                    bulkCachingHistory.put(tag, commands);
                }

                for (LynxDekaInterfaceCommand<?> otherCommand : commands) {
                    if (otherCommand.getDestModuleAddress() == command.getDestModuleAddress() &&
                            otherCommand.getCommandNumber() == command.getCommandNumber() &&
                            Arrays.equals(otherCommand.toPayloadByteArray(), command.toPayloadByteArray())) {
                        clearBulkCache();
                        break;
                    }
                }
            }

            if (lastBulkData == null) {
                getBulkData(); // populates lastBulkData with non-null value or throws
            }

            // recording the command must come after getBulkData() clears the cache
            if (bulkCachingMode == BulkCachingMode.AUTO) {
                commands.add(command);
            }

            return lastBulkData;
        }
    }

    /**
     * This method "wouldIssueBulkData", copied and subseqeuntly modified 
     * from "recordBulkCachingIntent", is subject to the following:
     * 
     * Copyright (c) 2016 Robert Atkinson
     * 
     * All rights reserved.
     * 
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     * 
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     * 
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     * 
     * Neither the name of Robert Atkinson nor the names of his contributors may be used to
     * endorse or promote products derived from this software without specific prior
     * written permission.
     * 
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
     * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
     * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */
    public boolean wouldIssueBulkData(LynxDekaInterfaceCommand<?> command, String tag) {
        warnIfClosed();
        synchronized (bulkCachingLock) {
            List<LynxDekaInterfaceCommand<?>> commands = bulkCachingHistory.get(tag);

            if (bulkCachingMode == BulkCachingMode.AUTO && commands != null) {
                // automatically clear the cache if necessary based on the command history
                for (LynxDekaInterfaceCommand<?> otherCommand : commands) {
                    if (otherCommand.getDestModuleAddress() == command.getDestModuleAddress() &&
                            otherCommand.getCommandNumber() == command.getCommandNumber() &&
                            Arrays.equals(otherCommand.toPayloadByteArray(), command.toPayloadByteArray())) {
                        // clearBulkCache(); // Ordinarily clears the bulk data cache.
                        return true;
                    }
                }
            }

            if (lastBulkData == null) {
                // System.out.println("dumness 800");
                return true;
            }

            return false;
        }
    }

}