/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *   (c) 2017 Muellerlab
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/optical_flow_report.h>
#include <uORB/topics/range_sensor_report.h>

#include "flowdeck.hpp"
#include "vl53l0x.hpp"

extern "C" __EXPORT int flow_main(int argc, char *argv[]);

static bool volatile thread_should_exit = false;
static bool volatile thread_running = false;
static int volatile daemon_task;

//UORB STUFF
struct optical_flow_report_s flowReport;
orb_advert_t pub_fd_flowReport = 0;
struct range_sensor_report_s rangeReport;
orb_advert_t pub_fd_rangeReport = 0;

int flow_thread_main(int argc, char *argv[]);

static FlowDeck flowdeck;
static VL53L0X rangeSensor;

void flow_usage();

void flow_usage() {
  printf("Optical flow  app, USAGE: range {start|stop|status|set|get}\n");
  printf(
      "\t Start the app as follows, where [VERBOSE] is an optional argument (0/1) to set whether to print info to the screen");
  printf("\t nsh> flow start [VERBOSE]\n");
}

volatile unsigned loopCounter = 0;

int flow_main(int argc, char *argv[]) {
  if (argc <= 1) {
    flow_usage();
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    thread_should_exit = true;
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      printf("Loop counter = %d", loopCounter);
      flowdeck.PrintStatus();
      rangeSensor.PrintStatus();
      printf("\n\n");
    } else {
      warnx("\tnot started\n");
    }

    return 0;
  }

  if (!strcmp(argv[1], "start")) {

    if (thread_running) {
      warnx("daemon already running\n");
      /* this is not an error */
      return 0;
    }

    thread_should_exit = false;
    daemon_task = px4_task_spawn_cmd(
        "flow_loop", SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        2000, flow_thread_main,
        (argv) ? (char * const *) &argv[2] : (char * const *) NULL);
    return 0;
  }

  flow_usage();
  return -1;
}

int flow_thread_main(int argc, char *argv[]) {
  bool success;

  success = flowdeck.Init();
  if (!success) {
    printf("Failed to init optical flow deck\n");
    return -1;
  }

  success = rangeSensor.Init();
  if (!success) {
    printf("Failed to init range sensor\n");
    return -1;
  }

  if (!pub_fd_flowReport) {
    memset(&flowReport, 0, sizeof(flowReport));
    pub_fd_flowReport = orb_advertise(ORB_ID(optical_flow_report), &flowReport);
    printf("Advertising uorb = %d\n", int(pub_fd_flowReport));
  }

  if (!pub_fd_rangeReport) {
    memset(&rangeReport, 0, sizeof(rangeReport));
    pub_fd_rangeReport = orb_advertise(ORB_ID(range_sensor_report),
                                       &rangeReport);
    printf("Advertising uorb = %d\n", int(pub_fd_rangeReport));
  }

  printf("### optical flow ###\n");

  thread_running = true;

  for (;;) {
    loopCounter++;
    flowdeck.Loop();
    rangeSensor.Loop();

    flowdeck.GetFlow(flowReport.flow_x, flowReport.flow_y);

    rangeReport.range = rangeSensor.GetRange();

    int res1 = orb_publish(ORB_ID(optical_flow_report), pub_fd_flowReport,
                           &flowReport);
    int res2 = orb_publish(ORB_ID(range_sensor_report), pub_fd_rangeReport,
                           &rangeReport);
    if (res1 < 0 or res2 < 0) {
      thread_running = false;
      printf("Error publishing to uorb = %d, %d\n", res1, res2);
      continue;
    }

    usleep(1000);
    if (thread_should_exit) {
      break;
    }

  }
  thread_running = false;
  return 0;
}
