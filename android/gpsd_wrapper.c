/*
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <android/log.h>
#include <cutils/properties.h>
#include <sys/system_properties.h>

int main (){
	char gpsd_params[PROP_VALUE_MAX];
	char cmd[1024];
	int cmd_len;
	int i = 0;

	property_get("service.gpsd.parameters", gpsd_params,
                     "-Nn,-D2,/dev/ttyACM0,/dev/ttyACM1");
	while (0 != gpsd_params[i]){
                // FIXME: gpsd_params are not checked for command injection
		if (gpsd_params[i] == ',') {
                    gpsd_params[i] = ' ';
                }
		i++;
	}
	cmd_len = snprintf(cmd, sizeof(cmd),
                          "/vendor/bin/logwrapper /vendor/bin/gpsd %s",
                          gpsd_params);
	if (0 > cmd_len || sizeof(cmd) <= (size_t)cmd_len) {
		__android_log_print(ANDROID_LOG_ERROR, "gpsd_wrapper",
                                    "gpsd command line is too long");
		return 1;
	}

	__android_log_print(ANDROID_LOG_DEBUG, "gpsd_wrapper",
                            "Starting gpsd: %s", cmd);

        // FIXME: gpsd_params are not checked for command injection
	system(cmd);
	return 0;
}

// vim: set expandtab shiftwidth=4
