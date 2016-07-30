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
#include "libavdevice/avdevice.h"
#include "openfile.h"
#include "ffmpeg.h"

void ff_log_callback(void *avcl, int level, const char *fmt, va_list vl) {
    FILE *fp = fopen("/Desktop/av_log.txt", "a+");
    if (fp) {
        vfprintf(fp, fmt, vl);
        fflush(fp);
        fclose(fp);
    }
}

int main(int argc, const char * argv[]) {
    // insert code here...
    av_register_all();
    avfilter_register_all();
    avformat_network_init();
    avdevice_register_all();
    
    AVDictionary *options = NULL;
    av_dict_set(&options, "framerate", "30", 0);
    
    open_input_file("0:1");
    MediaSource mediaSource = { NULL, NULL, NULL, NULL };
    mediaSource.audio_avfilter = av_strdup("anull");
    mediaSource.video_avfilter = av_strdup("null");
    open_output_file("rtmp://localhost/live/test", &mediaSource, 480, 320);
    transcode();
    return 0;
}
