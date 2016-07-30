//
//  main.c
//  ffplay
//
//  Created by wlanjie on 16/6/27.
//  Copyright © 2016年 com.wlanjie.ffplay. All rights reserved.
//

#include <stdio.h>
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavfilter/avfilter.h"
#include "ffplay.h"

void ff_log_callback(void *avcl, int level, const char *fmt, va_list vl) {
    char log[1024];
//    snprintf(log, sizeof(log), fmt, vl);
}

int main(int argc, const char * argv[]) {
    // insert code here...
    av_register_all();
    avfilter_register_all();
    avformat_network_init();
    av_log_set_callback(ff_log_callback);
//    int ret = init_ffplay("/Users/wlanjie/Desktop/sintel.mp4");
//    int ret = init_ffplay("rtmp://live.hkstv.hk.lxdns.com/live/hks");
//    int ret = init_ffplay("rtmp://192.168.0.68/live/test");
    int ret = init_ffplay("rtmp://192.168.1.103/live/test");
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "init_ffplay error %d\n", ret);
    }
    return 0;
}
