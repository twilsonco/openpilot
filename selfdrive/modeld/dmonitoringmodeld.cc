#include <sys/resource.h>
#include <limits.h>

#include <cstdio>
#include <cstdlib>

#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/modeld/models/dmonitoring.h"
#include "selfdrive/common/params.h"

ExitHandler do_exit;

void run_model(DMonitoringModelState &model, VisionIpcClient &vipc_client) {
  PubMaster pm({"driverState"});
  double last = 0;

  bool low_overhead_mode = Params().getBool("LowOverheadMode");
  int last_param_check_iter = 0;
  int param_check_interval_base = 200;
  int param_check_interval = param_check_interval_base;
  int iter = param_check_interval + 1;

  while (!do_exit) {
    iter++;
    if (iter > param_check_interval){
      low_overhead_mode = Params().getBool("LowOverheadMode");
      iter = 0;
    }
    if (low_overhead_mode){
      param_check_interval = 5;
      util::sleep_for(1000);
      continue;
    }
    else{
      param_check_interval = param_check_interval_base;
    }
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    double t1 = millis_since_boot();
    DMonitoringResult res = dmonitoring_eval_frame(&model, buf->addr, buf->width, buf->height);
    double t2 = millis_since_boot();

    // send dm packet
    dmonitoring_publish(pm, extra.frame_id, res, (t2 - t1) / 1000.0, model.output);

    //printf("dmonitoring process: %.2fms, from last %.2fms\n", t2 - t1, t1 - last);
    last = t1;
  }
}

int main(int argc, char **argv) {
  setpriority(PRIO_PROCESS, 0, -15);

  // init the models
  DMonitoringModelState model;
  dmonitoring_init(&model);

  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_YUV_FRONT, true);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  if (vipc_client.connected) {
    LOGW("connected with buffer size: %d", vipc_client.buffers[0].len);
    run_model(model, vipc_client);
  }

  dmonitoring_free(&model);
  return 0;
}
