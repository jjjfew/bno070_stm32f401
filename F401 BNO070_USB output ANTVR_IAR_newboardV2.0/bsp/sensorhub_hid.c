/****************************************************************************
* Copyright (C) 2015 Hillcrest Laboratories, Inc.
*
* Filename:
* Date:
* Description:
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use of such information nor for any infringement
* of patents or other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include "sensorhub_hid.h"
#include "sensorhub.h"
#include <string.h>

#define GET_REPORT_BUFSIZE (64)
#define SET_REPORT_BUFSIZE (64)
#define MAX_INPUT_REPORT_BUFSIZE (64)

int shhid_setReport(const sensorhub_t * sh,
                    sensorhub_ReportType_t reportType,
                    sensorhub_ReportId_t reportId,
                    const uint8_t * payload, uint8_t payloadLength)
{
    uint8_t cmd[32];
    int ix,p;

    cmd[0] = BNO070_REGISTER_COMMAND;
    cmd[1] = 0;

    if (reportId < 0xf) {
        cmd[2] = reportType | reportId;
        cmd[3] = HID_SET_REPORT_OPCODE;
        ix = 4;
    } else {
        cmd[2] = reportType | 0xf;
        cmd[3] = HID_SET_REPORT_OPCODE;
        cmd[4] = reportId;
        ix = 5;
    }

    cmd[ix++] = BNO070_REGISTER_DATA;
    cmd[ix++] = 0;

    cmd[ix++] = payloadLength + 2;
    cmd[ix++] = 0;

    memcpy(&cmd[ix], payload, payloadLength);
    ix += payloadLength;
    
    //for(p=0;p<ix;p++)printf("%02x ",cmd[p]); printf("\n");
    return sensorhub_i2cTransferWithRetry(sh, sh->sensorhubAddress, cmd, ix, NULL, 0);  
}

int shhid_getReport(const sensorhub_t * sh,
                    sensorhub_ReportType_t reportType,
                    sensorhub_ReportId_t reportId,
                    uint8_t * payload, uint8_t payloadLength)
{
    uint8_t cmd[7];
    int ix;

    cmd[0] = BNO070_REGISTER_COMMAND;
    cmd[1] = 0;

    if (reportId < 0xf) {
        cmd[2] = reportType | reportId;
        cmd[3] = HID_GET_REPORT_OPCODE;
        ix = 4;
    } else {
        cmd[2] = reportType | 0xf;
        cmd[3] = HID_GET_REPORT_OPCODE;
        cmd[4] = reportId;
        ix = 5;
    }

    cmd[ix++] = BNO070_REGISTER_DATA;
    cmd[ix++] = 0;

    return sensorhub_i2cTransferWithRetry(sh, sh->sensorhubAddress, cmd, ix, payload,
                                          payloadLength);
}
