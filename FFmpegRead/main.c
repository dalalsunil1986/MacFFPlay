//
//  main.c
//  FFmpegRead
//
//  Created by wlanjie on 2019/4/7.
//  Copyright © 2019 com.wlanjie.opengl. All rights reserved.
//

#include <stdio.h>
#include "ffplay.h"

void calculate_display_rect(SDL_Rect *rect, int scr_xleft, int scr_ytop, int scr_width, int scr_height, int pic_width,
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

void set_default_window_size(int width, int height, AVRational sar) {
    SDL_Rect rect;
    calculate_display_rect(&rect, 0, 0, INT_MAX, height, width, height, sar);
    default_width = rect.w;
    default_height = rect.h;
}

int video_open(VideoState *is, Frame *vp) {
    int w, h;
    if (vp && vp->width) {
        set_default_window_size(vp->width, vp->height, vp->sar);
    }
    if (screen_width) {
        w = screen_width;
        h = screen_height;
    } else {
        w = default_width;
        h = default_height;
    }
    if (!window) {
        int flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
        if (is_full_screen) {
            flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
        }
        window = SDL_CreateWindow("", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (window) {
            SDL_RendererInfo info;
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (renderer) {
                if (!SDL_GetRendererInfo(renderer, &info)) {
                    av_log(NULL, AV_LOG_VERBOSE, "Inilialized %s renderer.\n", info.name);
                }
            }
        }
    } else {
        SDL_SetWindowSize(window, w, h);
    }
    if (!window && !renderer) {
        av_log(NULL, AV_LOG_ERROR, "SDL: could not set video model - exiting\n");
        return AVERROR(ENOMEM);
    }
    is->width = w;
    is->height = h;
    return 0;
}

int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture) {
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

int upload_texture(SDL_Texture *tex, AVFrame *frame, struct SwsContext **img_convert_ctx) {
    int ret = 0;
    switch (frame->format) {
        case AV_PIX_FMT_YUV420P:
            ret = SDL_UpdateYUVTexture(tex, NULL, frame->data[0], frame->linesize[0],
                                       frame->data[1], frame->linesize[1],
                                       frame->data[2], frame->linesize[2]);
            break;
            
        case AV_PIX_FMT_BGRA:
            ret = SDL_UpdateTexture(tex, NULL, frame->data[0], frame->linesize[0]);
            break;
        default:
            /* This should only happed if we are not using avfilter */
            *img_convert_ctx = sws_getCachedContext(*img_convert_ctx,
                                                    frame->width, frame->height, frame->format, frame->width, frame->height,
                                                    AV_PIX_FMT_BGRA, SWS_BICUBIC, NULL, NULL, NULL);
            if (*img_convert_ctx != NULL) {
                uint8_t *pixels;
                int pitch;
                if (!SDL_LockTexture(tex, NULL, (void **) &pixels, &pitch)) {
                    sws_scale(*img_convert_ctx, (const uint8_t * const *) frame->data, frame->linesize,
                              0, frame->height, &pixels, &pitch);
                    SDL_UnlockTexture(tex);
                }
            } else {
                av_log(NULL, AV_LOG_FATAL, "Can't initialize the conversion context.\n");
                ret = -1;
            }
            break;
    }
    return ret;
}

extern inline int compute_mod(int a, int b);

inline int compute_mod(int a, int b) {
    return a < 0 ? a % b + b : a % b;
}

extern inline void fill_rectangle(int x, int y, int w, int h);

inline void fill_rectangle(int x, int y, int w, int h) {
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h) {
        SDL_RenderFillRect(renderer, &rect);
    }
}


void video_image_display(VideoState *is) {
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;
    vp = frame_queue_peek_last(&is->pictq);
    if (vp->bmp) {
        if (is->subtitle_st) {
            if (frame_queue_nb_remaining(&is->subpq) > 0) {
                sp = frame_queue_peek(&is->subpq);
                if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
                    if (!sp->uploaded) {
                        uint8_t *pixels;
                        int picth;
                        if (!sp->width || !sp->height) {
                            sp->width = vp->width;
                            sp->height = vp->height;
                        }
                        if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0) {
                            return;
                        }
                        for (int i = 0; i < sp->sub.num_rects; i++) {
                            AVSubtitleRect *sub_rect = sp->sub.rects[i];
                            sub_rect->x = av_clip(sub_rect->x, 0, sp->width);
                            sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
                            sub_rect->w = av_clip(sub_rect->w, 0, sp->width - sub_rect->x);
                            sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);
                            
                            is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx, sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
                                                                       sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA, 0, NULL, NULL, NULL);
                            if (!is->sub_convert_ctx) {
                                av_log(NULL, AV_LOG_FATAL, "Can't initialize the conversion context.\n");
                                return;
                            }
                            if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *) sub_rect, (void **)&pixels, &picth)) {
                                sws_scale(is->sub_convert_ctx, (const uint8_t * const *) sub_rect->data, sub_rect->linesize, 0, sub_rect->h, &pixels, &picth);
                                SDL_UnlockTexture(is->sub_texture);
                            }
                        }
                        sp->uploaded = 1;
                    }
                } else {
                    sp = NULL;
                }
            }
        }
        
        calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);
        
        if (!vp->uploaded) {
            if (upload_texture(vp->bmp, vp->frame, &is->img_convert_ctx) < 0) {
                return;
            }
            vp->uploaded = 1;
        }
        SDL_RenderCopy(renderer, vp->bmp, NULL, &rect);
        if (sp) {
#if USE_ONEPASS_SUBTITLE_RENDER
            SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
#else
            double xratio = (double) rect.w / (double) sp->width;
            double yratio = (double) rect.h / (double) sp->height;
            for (int i = 0; i < sp->sub.num_rects; i++) {
                SDL_Rect *sub_rect = (SDL_Rect *) sp->sub.rects[i];
                SDL_Rect target = {.x = rect.y + sub_rect->x * xratio,
                    .y = rect.y + sub_rect->y * yratio,
                    .w = sub_rect->w * xratio,
                    .h = sub_rect->h * yratio};
                SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target);
            }
#endif
        }
    }
}

void video_display(VideoState *is) {
    if (!window) {
        int result = video_open(is, NULL);
        if (result < 0) {
            //TODO not video open, exit player
        }
    }
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO) {
        //        video_audio_display(is);
    } else if (is->video_st) {
        video_image_display(is);
    }
    SDL_RenderPresent(renderer);
}

double vp_duration(VideoState *is, Frame *vp, Frame *nextvp) {
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

double compute_target_delay(double delay, VideoState *is) {
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

static void update_video_pts(VideoState *is, double pts, int64_t pos, int serial) {
    /* update current video pts */
    set_clock(&is->vidclk, pts, serial);
    sync_clock_to_slave(&is->extclk, &is->vidclk);
}

void video_retry(VideoState *is, double *remaining_time) {
    double time;
    Frame *sp, *sp2;
    if (frame_queue_nb_remaining(&is->pictq) == 0) {
        //nothing to do, no picture to display in the queue
    } else {
        double last_duration, delay;
        Frame *vp, *lastvp;
        
        /* dequeue the picture */
        lastvp = frame_queue_peek_last(&is->pictq);
        vp = frame_queue_peek(&is->pictq);
        
        if (vp->serial != is->videoq.serial) {
            frame_queue_next(&is->pictq);
            //TODO
            video_retry(is, remaining_time);
            return;
        }
        if (lastvp->serial != vp->serial) {
            is->frame_timer = av_gettime_relative() / 1000000.0;
        }
        if (is->paused) {
            /* display picture*/
            if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
                video_display(is);
                return;
            }
        }
        /* compute nominal last_duration */
        last_duration = vp_duration(is, lastvp, vp);
        delay = compute_target_delay(last_duration, is);
        
        time = av_gettime_relative() / 1000000.0;
        if (time < is->frame_timer + delay) {
            *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
            /* display picture*/
            if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
                video_display(is);
                return;
            }
        }
        is->frame_timer += delay;
        if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX) {
            is->frame_timer = time;
        }
        pthread_mutex_lock(&is->pictq.mutex);
        if (!isnan(vp->pts)) {
            update_video_pts(is, vp->pts, vp->pos, vp->serial);
        }
        pthread_mutex_unlock(&is->pictq.mutex);
        
        if (is->subtitle_st) {
            while (frame_queue_nb_remaining(&is->subpq) > 0) {
                sp = frame_queue_peek(&is->subpq);
                if (frame_queue_nb_remaining(&is->subpq) > 1) {
                    sp2 = frame_queue_peek_next(&is->subpq);
                } else {
                    sp2 = NULL;
                }
                if (sp->serial != is->subtitleq.serial ||
                    (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000))) ||
                    (sp2 && is->vidclk.pts > (sp2->pts + ((float)sp2->sub.start_display_time / 1000)))) {
                    if (sp->uploaded) {
                        for (int i = 0; i < sp->sub.num_rects; i++) {
                            AVSubtitleRect *sub_rect = sp->sub.rects[i];
                            uint8_t *pixels;
                            int pitch;
                            if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                for (int j = 0; j < sub_rect->h; j++, pixels += pitch) {
                                    memset(pixels, 0, sub_rect->w << 2);
                                }
                                SDL_UnlockTexture(is->sub_texture);
                            }
                        }
                    }
                    frame_queue_next(&is->subpq);
                } else {
                    break;
                }
            }
        }
        frame_queue_next(&is->pictq);
        is->force_refresh = 1;
        if (is->step && !is->paused) {
            stream_toggle_pause(is);
        }
    }
    if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
        video_display(is);
    }
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
    VideoState *is = opaque;
    double time;
    
    Frame *sp, *sp2;
    
    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK)
        check_external_clock_speed(is);
    
    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
        time = av_gettime_relative() / 1000000.0;
        if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
            video_display(is);
            is->last_vis_time = time;
        }
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }
    
    if (is->video_st) {
    retry:
        if (frame_queue_nb_remaining(&is->pictq) == 0) {
            // nothing to do, no picture to display in the queue
        } else {
            double last_duration, delay;
            Frame *vp, *lastvp;
            
            /* dequeue the picture */
            lastvp = frame_queue_peek_last(&is->pictq);
            vp = frame_queue_peek(&is->pictq);
            
            if (vp->serial != is->videoq.serial) {
                frame_queue_next(&is->pictq);
                goto retry;
            }
            
            if (lastvp->serial != vp->serial)
                is->frame_timer = av_gettime_relative() / 1000000.0;
            
            if (is->paused)
                goto display;
            
            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp);
            delay = compute_target_delay(last_duration, is);
            
            time= av_gettime_relative()/1000000.0;
            if (time < is->frame_timer + delay) {
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                goto display;
            }
            
            is->frame_timer += delay;
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
                is->frame_timer = time;
            
            pthread_mutex_lock(&is->pictq.mutex);
            if (!isnan(vp->pts))
                update_video_pts(is, vp->pts, vp->pos, vp->serial);
            pthread_mutex_unlock(&is->pictq.mutex);
            
            if (is->subtitle_st) {
                while (frame_queue_nb_remaining(&is->subpq) > 0) {
                    sp = frame_queue_peek(&is->subpq);
                    
                    if (frame_queue_nb_remaining(&is->subpq) > 1)
                        sp2 = frame_queue_peek_next(&is->subpq);
                    else
                        sp2 = NULL;
                    
                    if (sp->serial != is->subtitleq.serial
                        || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                        || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
                    {
                        if (sp->uploaded) {
                            int i;
                            for (i = 0; i < sp->sub.num_rects; i++) {
                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
                                uint8_t *pixels;
                                int pitch, j;
                                
                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
                                        memset(pixels, 0, sub_rect->w << 2);
                                    SDL_UnlockTexture(is->sub_texture);
                                }
                            }
                        }
                        frame_queue_next(&is->subpq);
                    } else {
                        break;
                    }
                }
            }
            
            frame_queue_next(&is->pictq);
            is->force_refresh = 1;
            
            if (is->step && !is->paused)
                stream_toggle_pause(is);
        }
    display:
        /* display picture */
        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
            video_display(is);
    }
    is->force_refresh = 0;
}

void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {
    double remaining_time = 0.0;
    SDL_PumpEvents();
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
        if (remaining_time > 0.0) {
            av_usleep(remaining_time * 1000000.0);
        }
        remaining_time = REFRESH_RATE;
        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh)) {
            video_refresh(is, &remaining_time);
        }
        SDL_PumpEvents();
    }
}

void alloc_picture(VideoState *is) {
    Frame *vp;
    int sdl_format;
    vp = &is->pictq.queue[is->pictq.windex];
    video_open(is, vp);
    
    if (vp->format == AV_PIX_FMT_YUV420P) {
        sdl_format = SDL_PIXELFORMAT_YV12;
    } else {
        sdl_format = SDL_PIXELFORMAT_ARGB8888;
    }
    
    if (realloc_texture(&vp->bmp, sdl_format, vp->width, vp->height, SDL_BLENDMODE_NONE, 0) < 0) {
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

void event_loop(VideoState *cur_stream) {
    SDL_Event event;
    for (;;) {
        refresh_loop_wait_event(cur_stream, &event);
        switch (event.type) {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                        
                    default:
                        break;
                }
                break;
                
            case SDLK_q:
                do_exit(cur_stream);
                break;
            case SDLK_SPACE:
                stream_toggle_pause(cur_stream);
                cur_stream->step = 0;
                break;
                
            case FF_ALLOC_EVENT:
                alloc_picture(event.user.data1);
                break;
            default:
                break;
        }
        
    }
}

int main(int argc, const char * argv[]) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER);
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);
    
    VideoState* is = init_ffplay("/Users/wlanjie/Desktop/other/not_play.mp4");
    
    event_loop(is);
    return 0;
}