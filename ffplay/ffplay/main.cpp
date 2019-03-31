//
//  main.cpp
//  ffplay
//
//  Created by wlanjie on 2018/10/23.
//  Copyright © 2018年 com.wlanjie.ffplay. All rights reserved.
//

#include <stdio.h>
#include "ff_read.h"

void ff_log_callback(void *avcl, int level, const char *fmt, va_list vl) {
    char log[1024];
    snprintf(log, 1024, "%s", fmt);
    char printLog[1024];
    vsnprintf(printLog, 1024, log, vl);
    printf("%s\n", printLog);
}

int main(int argc, const char* argv[]) {
    av_register_all();
    avformat_network_init();
//    av_log_set_callback(ff_log_callback);
    FFmpegRead read;
    read.init("/Users/wlanjie/Desktop/a.mp4");
    return 0;
}
