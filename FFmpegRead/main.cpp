//
//  main.cpp
//  FFmpegRead
//
//  Created by wlanjie on 2019/4/5.
//  Copyright © 2019 com.wlanjie.opengl. All rights reserved.
//

#include <iostream>

extern "C" {
#include "ffplay.h"
}

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
//    FFmpegRead read;
//    read.init("/Users/wlanjie/Desktop/a.mp4");
    
//    init();
//    av_start("/Users/wlanjie/Desktop/a.mp4", 0);
    init_ffplay("/Users/wlanjie/Desktop/other/not_play.mp4");
    while (1) {
        
    }
    return 0;
}
