/**
 * @file   utils/windows.c
 * @author Martin Pulec     <pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2019-2023 CESNET
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CESNET nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#include "config_unix.h"
#include "config_win32.h"
#endif

#include "utils/windows.h"

#ifdef _WIN32
#include <audioclient.h>
#include <objbase.h>
#include "debug.h"
#endif // defined _WIN32

bool com_initialize(bool *com_initialized, const char *err_prefix)
{
#ifdef _WIN32
        if (err_prefix == NULL) {
                err_prefix = "";
        }
        *com_initialized = false;
        // Initialize COM on this thread
        HRESULT result = CoInitializeEx(NULL, COINIT_MULTITHREADED);
        if (SUCCEEDED(result)) {
                *com_initialized = true;
                return true;
        }
        if (result == RPC_E_CHANGED_MODE) {
                log_msg(LOG_LEVEL_WARNING, "%sCOM already intiialized with a different mode!\n", err_prefix);
                return true;
        }
        log_msg(LOG_LEVEL_ERROR, "%sInitialize of COM failed - %s\n", err_prefix, hresult_to_str(result));
        return false;
#else
        (void) com_initialized, (void) err_prefix;
        return true;
#endif
}

void com_uninitialize(bool *com_initialized)
{
#ifdef _WIN32
        if (!*com_initialized) {
                return;
        }
        *com_initialized = false;
        CoUninitialize();
#else
        (void) com_initialized;
#endif
}

#ifdef _WIN32
const char *hresult_to_str(HRESULT res) {
        _Thread_local static char unknown[128];
        const char *errptr = NULL;

        HRESULT_GET_ERROR_COMMON(res, errptr)
        if (errptr) {
                return errptr;
        }

        switch (res) {
                case RPC_E_CHANGED_MODE: return "A previous call to CoInitializeEx specified different concurrency model for this thread."; break; \
                case CO_E_NOTINITIALIZED: return "CoInitialize has not been called.";
		case AUDCLNT_E_ALREADY_INITIALIZED: return "The IAudioClient object is already initialized.";
		case AUDCLNT_E_WRONG_ENDPOINT_TYPE: return "The AUDCLNT_STREAMFLAGS_LOOPBACK flag is set but the endpoint device is a capture device, not a rendering device.";
		case AUDCLNT_E_BUFFER_SIZE_NOT_ALIGNED: return "The requested buffer size is not aligned. This code can be returned for a render or a capture device if the caller specified AUDCLNT_SHAREMODE_EXCLUSIVE and the AUDCLNT_STREAMFLAGS_EVENTCALLBACK flags. The caller must call Initialize again with the aligned buffer size.";
		case AUDCLNT_E_BUFFER_SIZE_ERROR: return "Indicates that the buffer duration value requested by an exclusive-mode client is out of range. The requested duration value for pull mode must not be greater than 500 milliseconds; for push mode the duration value must not be greater than 2 seconds.";
		case AUDCLNT_E_CPUUSAGE_EXCEEDED: return "Indicates that the process-pass duration exceeded the maximum CPU usage. The audio engine keeps track of CPU usage by maintaining the number of times the process-pass duration exceeds the maximum CPU usage. The maximum CPU usage is calculated as a percent of the engine's periodicity. The percentage value is the system's CPU throttle value (within the range of 10%% and 90%%). If this value is not found, then the default value of 40%% is used to calculate the maximum CPU usage.)";
		case AUDCLNT_E_DEVICE_INVALIDATED: return "The audio endpoint device has been unplugged, or the audio hardware or associated hardware resources have been reconfigured, disabled, removed, or otherwise made unavailable for use.";
		case AUDCLNT_E_DEVICE_IN_USE: return "The endpoint device is already in use. Either the device is being used in exclusive mode, or the device is being used in shared mode and the caller asked to use the device in exclusive mode.";
		case AUDCLNT_E_ENDPOINT_CREATE_FAILED: return "The method failed to create the audio endpoint for the render or the capture device. This can occur if the audio endpoint device has been unplugged, or the audio hardware or associated hardware resources have been reconfigured, disabled, removed, or otherwise made unavailable for use.";
		case AUDCLNT_E_INVALID_DEVICE_PERIOD: return "Indicates that the device period requested by an exclusive-mode client is greater than 500 milliseconds.";
		case AUDCLNT_E_UNSUPPORTED_FORMAT: return "The audio engine (shared mode) or audio endpoint device (exclusive mode) does not support the specified format.";
		case AUDCLNT_E_EXCLUSIVE_MODE_NOT_ALLOWED: return "The caller is requesting exclusive-mode use of the endpoint device, but the user has disabled exclusive-mode use of the device.";
		case AUDCLNT_E_BUFDURATION_PERIOD_NOT_EQUAL: return "The AUDCLNT_STREAMFLAGS_EVENTCALLBACK flag is set but parameters hnsBufferDuration and hnsPeriodicity are not equal.";
		case AUDCLNT_E_SERVICE_NOT_RUNNING: return "The Windows audio service is not running.";
                default:
                       snprintf(unknown, sizeof unknown, "(unknown: %ld S=%lu,R=%lu,C=%lu,facility=%lu,code=%lu)", res, ((unsigned long int) res) >> 31u, (res >> 30u) & 0x1, (res >> 29u) & 0x1, (res >> 16u) & 0x7ff, res & 0xff);
                       return unknown;
        }
}

/**
 * @param error   error code (typically GetLastError())
 *
 * @note returned error string is _not_ <NL>-terminated (stripped from FormatMessage)
 *
 * To achieve localized output, use languageid `MAKELANGID (LANG_NEUTRAL, SUBLANG_NEUTRAL)` and
 * FormatMessageW (with wchar_t), converted to UTF-8 by win_wstr_to_str.
*/
const char *get_win_error(DWORD error) {
        _Thread_local static char buf[1024] = "(unknown)";
        if (FormatMessageA (FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,   // flags
                                NULL,                // lpsource
                                error,                   // message id
                                MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US),    // languageid
                                buf, // output buffer
                                sizeof buf, // size of msgbuf, bytes
                                NULL)               // va_list of arguments
           ) {
                char *end = buf + strlen(buf) - 1;
                if (end > buf && *end == '\n') { // skip LF
                        *end-- = '\0';
                }
                if (end > buf && *end == '\r') { // skip CR
                        *end-- = '\0';
                }
                return buf;
        }

        snprintf(buf, sizeof buf, "[Could not find a description for error # %#lx.]", error);

        return buf;
}

/**
 * requires `SetConsoleOutputCP(CP_UTF8);` (done in common_preinit()) or passing the code page as terminal is set to
 * (command chcp)
 */
const char *win_wstr_to_str(const wchar_t *wstr) {
        _Thread_local static char res[1024];
        int ret = WideCharToMultiByte(CP_UTF8, 0, wstr, -1 /* NULL-terminated */, res, sizeof res - 1, NULL, NULL);
        if (ret == 0) {
                if (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
                        log_msg(LOG_LEVEL_ERROR, "win_wstr_to_str: Insufficient buffer length %zd, please report to %s!\n", sizeof res, PACKAGE_BUGREPORT);
                } else {
                        log_msg(LOG_LEVEL_ERROR, "win_wstr_to_str: %s\n", get_win_error(GetLastError()));
                }
        }
        return res;
}
#endif // defined _WIN32

/* vim: set expandtab sw=8 tw=120: */
