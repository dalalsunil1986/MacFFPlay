//
//  main.cpp
//  FFmpegRead
//
//  Created by wlanjie on 2019/4/5.
//  Copyright © 2019 com.wlanjie.opengl. All rights reserved.
//

#include <iostream>
#include "SDL.h"

extern "C" {
#include "ffplay.h"
}

static SDL_Window *window_;
static SDL_Renderer* render_;

void calculateDisplayRect(SDL_Rect *rect, int scr_xleft, int scr_ytop, int scr_width, int scr_height, int pic_width,
                            int pic_height, AVRational pic_sar) {
    float aspect_ratio;
    int width, height, x, y;
    if (pic_sar.num == 0) {
        aspect_ratio = 0;
    } else {
        aspect_ratio = av_q2d(pic_sar);
    }
    if (aspect_ratio <= 0.0) {
        aspect_ratio = 1.0;
    }
    aspect_ratio *= (float) pic_width / (float) pic_height;
    
    /* XXX: we suppose the screen has a 1.0 pixel ratio */
    height = scr_height;
    width = lrint(height * aspect_ratio) & ~1;
    if (width > scr_width) {
        width = scr_width;
        height = lrint(width / aspect_ratio) & ~1;
    }
    x = (scr_width - width) / 2;
    y = (scr_height - height) / 2;
    rect->x = scr_xleft + x;
    rect->y = scr_ytop + y;
    rect->w = FFMAX(width, 1);
    rect->h = FFMAX(height, 1);
}

void setDefaultWindowSize(int width, int height, AVRational sar) {
    SDL_Rect rect;
    calculateDisplayRect(&rect, 0, 0, INT_MAX, height, width, height, sar);
    default_width = rect.w;
    default_height = rect.h;
}

int videoOpen(VideoState* is, Frame* vp) {
    int w = 640, h = 480;
    if (vp && vp->width) {
        setDefaultWindowSize(vp->width, vp->height, vp->sar);
    }
    if (!window_) {
        int flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
        window_ = SDL_CreateWindow("ffplay", 0, 0, w, h, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (window_) {
            SDL_RendererInfo info;
            render_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (render_) {
                if (!SDL_GetRendererInfo(render_, &info)) {
                    printf("create render error\n");
                }
            }
        }
    } else {
        SDL_SetWindowSize(window_, w, h);
    }
    is->width = w;
    is->height = h;
    return 0;
}

int uploadTexture(SDL_Texture* texture, AVFrame* frame, struct SwsContext **img_convert_ctx) {
    int ret = 0;
    switch (frame->format) {
        case AV_PIX_FMT_YUV420P:
            ret = SDL_UpdateYUVTexture(texture, NULL, frame->data[0], frame->linesize[0],
                                       frame->data[1], frame->linesize[1],
                                       frame->data[2], frame->linesize[2]);
            break;
            
        case AV_PIX_FMT_BGRA:
            ret = SDL_UpdateTexture(texture, NULL, frame->data[0], frame->linesize[0]);
            break;
            
        default:
            /* This should only happed if we are not using avfilter */
            *img_convert_ctx = sws_getCachedContext(*img_convert_ctx,
                                                    frame->width, frame->height, ( AVPixelFormat) frame->format, frame->width, frame->height,
                                                    (AVPixelFormat) AV_PIX_FMT_BGRA, SWS_BICUBIC, NULL, NULL, NULL);
            if (*img_convert_ctx != NULL) {
                uint8_t *pixels;
                int pitch;
                if (!SDL_LockTexture(texture, NULL, (void **) &pixels, &pitch)) {
                    sws_scale(*img_convert_ctx, (const uint8_t * const *) frame->data, frame->linesize,
                              0, frame->height, &pixels, &pitch);
                    SDL_UnlockTexture(texture);
                }
            } else {
                av_log(NULL, AV_LOG_FATAL, "Can't initialize the conversion context.\n");
                ret = -1;
            }
            break;
    }
    return ret;
}

void videoImageDisplay(VideoState* is) {
    Frame* vp = frame_queue_peek_last(&is->pictq);
    SDL_Rect rect;
    if (vp->bmp) {
        calculateDisplayRect(&rect, 0, 0, 640, 480, vp->width, vp->height, vp->sar);
        if (!vp->uploaded) {
            if (uploadTexture(vp->bmp, vp->frame, &is->img_convert_ctx) < 0) {
                return;
            }
            vp->uploaded = 1;
        }
        SDL_RenderCopy(render_, vp->bmp, NULL, &rect);
    }
}

void videoDisplay(VideoState* is) {
    if (window_ != nullptr) {
        int result = videoOpen(is, NULL);
        if (result != 0) {
            
        }
    }
    SDL_SetRenderDrawColor(render_, 0, 0, 0, 255);
    SDL_RenderClear(render_);
    if (is->video_st) {
        videoImageDisplay(is);
    }
    SDL_RenderPresent(render_);
}

double vpDuration(VideoState *is, Frame *vp, Frame *nextvp) {
    if (vp->serial == nextvp->serial) {
        double duration = nextvp->pts - vp->pts;
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration) {
            return vp->duration;
        } else {
            return duration;
        }
    } else {
        return 0.0;
    }
}

double computeTargetDelay(double delay, VideoState *is) {
    double sync_threshold, diff = 0;
    
    /* update delay to follow master synchronisation source */
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* if video is slave, we try to correct big delays by
         duplicating or deleting a frame */
        diff = get_clock(&is->vidclk) - get_master_clock(is);
        
        /* skip or repeat frame. We take into account the
         delay to compute the threshold. I still don't know
         if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX, delay));
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
            if (diff <= -sync_threshold)
                delay = FFMAX(0, delay + diff);
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD)
                delay = delay + diff;
            else if (diff >= sync_threshold)
                delay = 2 * delay;
        }
    }
    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n", delay, -diff);
    return delay;
}

void updateVideoPts(VideoState* is, double pts, int64_t pos, int serial) {
    set_clock(&is->vidclk, pts, serial);
    sync_clock_to_slave(&is->extclk, &is->vidclk);
}

void videoRefresh(void *opaque, double *remaining_time) {
    VideoState* is = (VideoState*) opaque;
    double time;
    
//    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK)
//        check_external_clock_speed(is);
//
    if (is->video_st) {
        double last_duration, delay;
        Frame* vp, *lastvp;
        lastvp = frame_queue_peek_last(&is->pictq);
        vp = frame_queue_peek(&is->pictq);
        
        if (vp->serial != is->videoq.serial) {
            frame_queue_next(&is->pictq);
            videoRefresh(is, remaining_time);
            return;
        }
        
        if (lastvp->serial != vp->serial) {
            is->frame_timer = av_gettime_relative() / 1000000.0;
        }
        
        if (is->paused) {
            videoDisplay(is);
            return;
        }
        
        last_duration = vpDuration(is, lastvp, vp);
        delay = computeTargetDelay(last_duration, is);
        time = av_gettime_relative() / 1000000.0;
        if (time < is->frame_timer + delay) {
            *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
            videoDisplay(is);
            return;
        }
        
        is->frame_timer += delay;
        if (delay > 0 && time - is->frame_timer > AV_SYNC_FRAMEDUP_THRESHOLD) {
            is->frame_timer = time;
        }
        
        pthread_mutex_lock(&is->pictq.mutex);
        if (!isnan(vp->pts)) {
            updateVideoPts(is, vp->pts, vp->pos, vp->serial);
        }
        pthread_mutex_unlock(&is->pictq.mutex);
        
        frame_queue_next(&is->pictq);
        is->force_refresh = 1;
    }
}

void refreshLoopWaitEvent(VideoState* is, SDL_Event* event) {
    double remainig_time = 0.0;
    SDL_PumpEvents();
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
        if (remainig_time > 0.0) {
            av_usleep(remainig_time * 1000000.0);
        }
        remainig_time = REFRESH_RATE;
        if ((!is->paused || is->force_refresh)) {
            videoRefresh(is, &remainig_time);
        }
        SDL_PumpEvents();
    }
}

int reallocTexture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture) {
    Uint32 format;
    int access, w, h;
    if (SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 || new_width != w || new_height != h || new_format != format) {
        void *pixels;
        int picth;
        SDL_DestroyTexture(*texture);
        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height))) {
            av_log(NULL, AV_LOG_ERROR, "SDL_CreateTexture(): %s\n", SDL_GetError());
            return -1;
        }
        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0) {
            return -1;
        }
        if (init_texture) {
            if (SDL_LockTexture(*texture, NULL, &pixels, &picth) < 0) {
                return -1;
            }
            memset(pixels, 0, picth * new_height);
            SDL_UnlockTexture(*texture);
        }
    }
    return 0;
}

void allocPicture(VideoState *is) {
    Frame *vp;
    int sdl_format;
    vp = &is->pictq.queue[is->pictq.windex];
    videoOpen(is, vp);
    
    if (vp->format == AV_PIX_FMT_YUV420P) {
        sdl_format = SDL_PIXELFORMAT_YV12;
    } else {
        sdl_format = SDL_PIXELFORMAT_ARGB8888;
    }
    
    if (reallocTexture(&vp->bmp, sdl_format, vp->width, vp->height, SDL_BLENDMODE_NONE, 0) < 0) {
        //TODO exit application
        /* SDL allocates a buffer smaller than requested if the video
         * overlay hardware is unable to support the requested size. */
        av_log(NULL, AV_LOG_FATAL,
               "Error: the video system does not support an image\n"
               "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
               "to reduce the image size.\n", vp->width, vp->height );
        
    }
    pthread_mutex_lock(&is->pictq.mutex);
    vp->allocated = 1;
    pthread_cond_signal(&is->pictq.cond);
    pthread_mutex_unlock(&is->pictq.mutex);
}

void eventLoop(VideoState* vs) {
    SDL_Event event;
    for (;;) {
        refreshLoopWaitEvent(vs, &event);
        switch (event.type) {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                        
                    default:
                        break;
                }
                break;
                
//            case SDLK_q:
//                do_exit(vs);
//                break;
//            case SDLK_SPACE:
//                stream_toggle_pause(vs);
//                cur_stream->step = 0;
//                break;
//
            case FF_ALLOC_EVENT:
                allocPicture((VideoState*) event.user.data1);
                break;
            default:
                break;
        }
    }
}

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
//    FFmpegRead read;
//    read.init("/Users/wlanjie/Desktop/a.mp4");
    
//    init();
//    av_start("/Users/wlanjie/Desktop/a.mp4", 0);
    VideoState* vs = init_ffplay("/Users/wlanjie/Desktop/other/not_play.mp4");
    
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER);
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);
    
    eventLoop(vs);
    return 0;
}
