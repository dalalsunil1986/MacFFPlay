//
//  ffplay.c
//  ffplay
//
//  Created by wlanjie on 16/6/27.
//  Copyright © 2016年 com.wlanjie.ffplay. All rights reserved.
//

#include "ffplay.h"

#pragma clang diagnostic ignored "-Wdeprecated-declarations"

int frame_queue_init( FrameQueue *f, PacketQueue *pktq, int max_size, int keep_last) {
    memset(f, 0, sizeof(FrameQueue));
    int result = pthread_mutex_init(&f->mutex, NULL);
    if (result != 0) {
        av_log(NULL, AV_LOG_FATAL, "Frame Init SDL_CreateMutex(): %d\n", result);
        return AVERROR(ENOMEM);
    }
    result = pthread_cond_init(&f->cond, NULL);
    if (result != 0) {
        av_log(NULL, AV_LOG_FATAL, "Frame Init SDL_CreateCond(): %d\n", result);
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = max_size;
    f->keep_last = keep_last;
    for (int i = 0; i < f->max_size; i++) {
        if (!(f->queue[i].frame = av_frame_alloc())) {
            return AVERROR(ENOMEM);
        }
    }
    return 0;
}

Frame *frame_queue_peek_writable(FrameQueue *f) {
    pthread_mutex_lock(&f->mutex);
    while (f->size >= f->max_size && !f->pktq->abort_request) {
        pthread_cond_wait(&f->cond, &f->mutex);
    }
    pthread_mutex_unlock(&f->mutex);
    if (f->pktq->abort_request) {
        return NULL;
    }
    return &f->queue[f->windex];
}

Frame *frame_queue_peek_readable(FrameQueue *f) {
    pthread_mutex_lock(&f->mutex);
    while (f->size - f->rindex_shown <= 0 && !f->pktq->abort_request) {
        pthread_cond_wait(&f->cond, &f->mutex);
    }
    pthread_mutex_unlock(&f->mutex);
    if (f->pktq->abort_request) {
        return NULL;
    }
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

void frame_queue_push(FrameQueue *f) {
    if (++f->windex == f->max_size) {
        f->windex = 0;
    }
    pthread_mutex_lock(&f->mutex);
    f->size++;
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

Frame *frame_queue_peek(FrameQueue *f) {
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

/* return the number of undisplayed frames in the queue */
int frame_queue_nb_remaining(FrameQueue *f) {
    return f->size - f->rindex_shown;
}

Frame *frame_queue_peek_last(FrameQueue *f) {
    return &f->queue[f->rindex];
}

Frame *frame_queue_peek_next(FrameQueue *f) {
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

void frame_queue_unref_item(Frame *vp) {
    av_frame_unref(vp->frame);
    avsubtitle_free(&vp->sub);
}

void frame_queue_next(FrameQueue *f) {
    if (f->keep_last && !f->rindex_shown) {
        f->rindex_shown = 1;
        return;
    }
    frame_queue_unref_item(&f->queue[f->rindex]);
    if (++f->rindex == f->max_size) {
        f->rindex = 0;
    }
    pthread_mutex_lock(&f->mutex);
    f->size--;
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

#ifdef __APPLE__
void free_picture(Frame *vp) {
    if (vp->bmp) {
        SDL_DestroyTexture(vp->bmp);
        vp->bmp = NULL;
    }
}
#endif

void frame_queue_signal(FrameQueue *f) {
    pthread_mutex_lock(&f->mutex);
    pthread_cond_signal(&f->cond);
    pthread_mutex_unlock(&f->mutex);
}

void frame_queue_destory(FrameQueue *f) {
    int i;
    for (i = 0; i < f->max_size; i++) {
        Frame *vp = &f->queue[i];
        frame_queue_unref_item(vp);
        av_frame_free(&vp->frame);
#ifdef __APPLE__
        free_picture(vp);
#endif
    }
    pthread_mutex_destroy(&f->mutex);
    pthread_cond_destroy(&f->cond);
}

int packet_queue_init(PacketQueue *q) {
    memset(q, 0, sizeof(PacketQueue));
    int result = pthread_mutex_init(&q->mutex, NULL);
    if (result != 0) {
        av_log(NULL, AV_LOG_FATAL, "Packet Init SDL_CreateMutex(): %d\n", result);
        return AVERROR(ENOMEM);
    }
    result = pthread_cond_init(&q->cond, NULL);
    if (result != 0) {
        av_log(NULL, AV_LOG_FATAL, "Packet Init SDL_CreateCond(): %d\n", result);
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

int packet_queue_put_private(PacketQueue *q, AVPacket *packet) {
    MyAVPacketList *pkt1;
    if (q->abort_request) {
        return -1;
    }
    pkt1 = av_malloc(sizeof(MyAVPacketList));
    if (!pkt1) {
        return AVERROR(ENOMEM);
    }
    pkt1->pkt = *packet;
    pkt1->next = NULL;
    if (packet == &flush_pkt) {
        q->serial++;
    }
    pkt1->serial = q->serial;
    if (!q->last_pkt) {
        q->first_pkt = pkt1;
    } else {
        q->last_pkt->next = pkt1;
    }
    q->last_pkt = pkt1;
    q->nb_packets++;
    q->size += pkt1->pkt.size + sizeof(*pkt1);
    pthread_cond_signal(&q->cond);
    return 0;
}

void packet_queue_start(PacketQueue *q) {
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 0;
    packet_queue_put_private(q, &flush_pkt);
    pthread_mutex_unlock(&q->mutex);
}

void packet_queue_abort(PacketQueue *q) {
    pthread_mutex_lock(&q->mutex);
    q->abort_request = 1;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

void packet_queue_flush(PacketQueue *q) {
    MyAVPacketList *pkt, *pkt1;
    pthread_mutex_lock(&q->mutex);
    for (pkt = q->first_pkt; pkt; pkt = pkt1) {
        pkt1 = pkt->next;
        av_packet_unref(&pkt->pkt);
        av_freep(&pkt);
    }
    q->last_pkt = NULL;
    q->first_pkt = NULL;
    q->nb_packets = 0;
    q->size = 0;
    pthread_mutex_unlock(&q->mutex);
}

void packet_queue_destroy(PacketQueue *q) {
    packet_queue_flush(q);
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

void set_clock_at(Clock *c, double pts, int serial, double time) {
    c->pts = pts;
    c->last_update = time;
    c->pts_drift = c->pts - time;
    c->serial = serial;
}

void set_clock(Clock *c, double pts, int serial) {
    double time = av_gettime_relative() / 1000000.0;
    set_clock_at(c, pts, serial, time);
}

double get_clock(Clock *c) {
    if (*c->queue_serial != c->serial) {
        return NAN;
    }
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0;
        return c->pts_drift + time - (time - c->last_update) * (1.0 - c->speed);
    }
}

void set_clock_speed(Clock *c, double speed) {
    set_clock(c, get_clock(c), c->serial);
    c->speed = speed;
}

void sync_clock_to_slave(Clock *c, Clock *slave) {
    double clock = get_clock(c);
    double slave_clock = get_clock(slave);
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD)) {
        set_clock(c, slave_clock, slave->serial);
    }
}

void init_clock(Clock *clock, int *serial) {
    clock->speed = 1.0;
    clock->paused = 0;
    clock->queue_serial = serial;
    set_clock(clock, NAN, -1);
}

int decode_interrupt_cb(void *ctx) {
    VideoState *is = ctx;
    return is->abort_request;
}

int get_master_sync_type(VideoState *is) {
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st) {
            return AV_SYNC_VIDEO_MASTER;
        } else {
            return AV_SYNC_AUDIO_MASTER;
        }
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st) {
            return AV_SYNC_AUDIO_MASTER;
        } else {
            return AV_SYNC_EXTERNAL_CLOCK;
        }
    } else {
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}


double get_master_clock(VideoState *is) {
    double val;
    switch (get_master_sync_type(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
            
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    return val;
}

#ifdef __APPLE__
//void calculate_display_rect(SDL_Rect *rect, int scr_xleft, int scr_ytop, int scr_width, int scr_height, int pic_width,
//                            int pic_height, AVRational pic_sar) {
//    float aspect_ratio;
//    int width, height, x, y;
//    if (pic_sar.num == 0) {
//        aspect_ratio = 0;
//    } else {
//        aspect_ratio = av_q2d(pic_sar);
//    }
//    if (aspect_ratio <= 0.0) {
//        aspect_ratio = 1.0;
//    }
//    aspect_ratio *= (float) pic_width / (float) pic_height;
//    
//    /* XXX: we suppose the screen has a 1.0 pixel ratio */
//    height = scr_height;
//    width = lrint(height * aspect_ratio) & ~1;
//    if (width > scr_width) {
//        width = scr_width;
//        height = lrint(width / aspect_ratio) & ~1;
//    }
//    x = (scr_width - width) / 2;
//    y = (scr_height - height) / 2;
//    rect->x = scr_xleft + x;
//    rect->y = scr_ytop + y;
//    rect->w = FFMAX(width, 1);
//    rect->h = FFMAX(height, 1);
//}
//
//void set_default_window_size(int width, int height, AVRational sar) {
//    SDL_Rect rect;
//    calculate_display_rect(&rect, 0, 0, INT_MAX, height, width, height, sar);
//    default_width = rect.w;
//    default_height = rect.h;
//}
#endif

int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph, AVFilterContext *source_ctx, AVFilterContext *sink_ctx) {
    int ret;
    int nb_filters = graph->nb_filters;
    AVFilterInOut *outputs = NULL, *inputs = NULL;
    if (filtergraph) {
        outputs = avfilter_inout_alloc();
        inputs = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            avfilter_inout_free(&outputs);
            avfilter_inout_free(&inputs);
            return AVERROR(ENOMEM);
        }
        outputs->name = av_strdup("in");
        outputs->filter_ctx = source_ctx;
        outputs->pad_idx = 0;
        outputs->next = NULL;
        
        inputs->name = av_strdup("out");
        inputs->filter_ctx = sink_ctx;
        inputs->pad_idx = 0;
        inputs->next = NULL;
        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0) {
            avfilter_inout_free(&outputs);
            avfilter_inout_free(&inputs);
            av_err2str(ret);
        }
    } else {
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0) {
            avfilter_inout_free(&outputs);
            avfilter_inout_free(&inputs);
            av_err2str(ret);
        }
    }
    for (int i = 0; i < graph->nb_filters - nb_filters; i++) {
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);
    }
    ret = avfilter_graph_config(graph, NULL);
    if (ret < 0) {
        avfilter_inout_free(&outputs);
        avfilter_inout_free(&inputs);
        av_err2str(ret);
    }
    return ret;
}

int configure_audio_filters(VideoState *is, const char *afilters, int force_outout_format) {
    int ret;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    int sample_rates[2] = { 0, -1 };
    int64_t channel_layouts[2] = { 0, -1 };
    int channels[2] = { 0, -1 };
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
    char asrc_args[256];
    
    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc())) {
        return AVERROR(ENOMEM);
    }
    ret = snprintf(asrc_args, sizeof(asrc_args), "sample_rate=%d:sample_fmt=%s:channels=%d:time_base=%d/%d",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   is->audio_filter_src.channels, 1, is->audio_filter_src.freq);
    if (is->audio_filter_src.channel_layout) {
        snprintf(asrc_args + ret, sizeof(asrc_args) - ret, ":channel_layout=0x%"PRIx64, is->audio_filter_src.channel_layout);
    }
    ret = avfilter_graph_create_filter(&filt_asrc, avfilter_get_by_name("abuffer"), "ffplay_abuffer", asrc_args, NULL, is->agraph);
    if (ret < 0) {
        avfilter_graph_free(&is->agraph);
        av_err2str(ret);
        return ret;
    }
    ret = avfilter_graph_create_filter(&filt_asink, avfilter_get_by_name("abuffersink"), "ffplay_abuffersink", NULL, NULL, is->agraph);
    if (ret < 0) {
        avfilter_graph_free(&is->agraph);
        av_err2str(ret);
        return ret;
    }
    if ((ret = av_opt_set_int_list(filt_asink, "sample_fmts", sample_fmts, AV_SAMPLE_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0) {
        avfilter_graph_free(&is->agraph);
        av_err2str(ret);
        return ret;
    }
    if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 1, AV_OPT_SEARCH_CHILDREN)) < 0) {
        avfilter_graph_free(&is->agraph);
        av_err2str(ret);
        return ret;
    }
    if (force_outout_format) {
        channel_layouts[0] = is->audio_filter_src.channel_layout;
        channels[0] = is->audio_filter_src.channels;
        sample_rates[0] = is->audio_filter_src.freq;
        if ((ret = av_opt_set_int_list(filt_asink, "channel_layouts", channel_layouts, -1, AV_OPT_SEARCH_CHILDREN)) < 0) {
            avfilter_graph_free(&is->agraph);
            av_err2str(ret);
            return ret;
        }
        if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0, AV_OPT_SEARCH_CHILDREN)) < 0) {
            avfilter_graph_free(&is->agraph);
            av_err2str(ret);
            return ret;
        }
        if ((ret = av_opt_set_int_list(filt_asink, "channel_counts", channels, -1, AV_OPT_SEARCH_CHILDREN)) < 0) {
            avfilter_graph_free(&is->agraph);
            av_err2str(ret);
            return ret;
        }
        if ((ret = av_opt_set_int_list(filt_asink, "sample_rates", sample_rates, -1, AV_OPT_SEARCH_CHILDREN)) < 0) {
            avfilter_graph_free(&is->agraph);
            av_err2str(ret);
            return ret;
        }
    }
    if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc, filt_asink)) < 0) {
        avfilter_graph_free(&is->agraph);
        return ret;
    }
    is->in_audio_filter = filt_asrc;
    is->out_audio_filter = filt_asink;
    return ret;
}

#ifdef __APPLE__
/* copy samples for viewing in editor window */
void update_sample_display(VideoState *is, short *samples, int sample_size) {
    int size, len;
    size = sample_size / sizeof(short);
    while (size > 0) {
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        if (len > size)
            len = size;
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE) {
            is->sample_array_index = 0;
        }
        size -= len;
    }
}
#endif

/* return the wanted number of samples to get better sync if sync_type is video
 * or external master clock */
int synchronize_audio(VideoState *is, int nb_samples) {
    int wanted_nb_samples = nb_samples;
    
    /* if not master, then we try to remove or add samples to correct the clock */
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER) {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;
        
        diff = get_clock(&is->audclk) - get_master_clock(is);
        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD) {
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++;
            } else {
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);
                
                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    wanted_nb_samples = nb_samples + (int) (diff * is->audio_src.freq);
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    wanted_nb_samples = av_clip_c(wanted_nb_samples, min_nb_samples, max_nb_samples);
                }
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                       diff, avg_diff, wanted_nb_samples - nb_samples,
                       is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            /* too big difference : may be initial PTS errors, so reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum = 0;
        }
    }
    return wanted_nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 */
int audio_decoder_frame(VideoState *is) {
    int data_size, resample_data_size;
    int64_t dec_channel_layout;
    av_unused double audio_clock0;
    int wanted_nb_sample;
    Frame *af;
    
    if (is->paused) {
        return -1;
    }
    do {
        if (!(af = frame_queue_peek_readable(&is->sampq))) {
            return -1;
        }
        frame_queue_next(&is->sampq);
    } while (af->serial != is->audioq.serial);
    
    data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(af->frame), af->frame->nb_samples,
                                           af->frame->format, 1);
    
    dec_channel_layout = (af->frame->channel_layout && av_frame_get_channels(af->frame) == av_get_channel_layout_nb_channels(af->frame->channel_layout)) ?
                            af->frame->channel_layout : av_get_default_channel_layout(av_frame_get_channels(af->frame));
    wanted_nb_sample = synchronize_audio(is, af->frame->nb_samples);
    
    if (af->frame->format != is->audio_src.fmt ||
        dec_channel_layout != is->audio_src.channel_layout ||
        af->frame->sample_rate != is->audio_src.freq ||
        (wanted_nb_sample != af->frame->nb_samples && !is->swr_ctx)) {
        swr_free(&is->swr_ctx);
        is->swr_ctx = swr_alloc_set_opts(NULL, is->audio_tgt.channel_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                                         dec_channel_layout, af->frame->format, af->frame->sample_rate, 0, NULL);
        if (!is->swr_ctx || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                   af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format), av_frame_get_channels(af->frame),
                   is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        is->audio_src.channel_layout = dec_channel_layout;
        is->audio_src.channels = av_frame_get_channels(af->frame);
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }
    if (is->swr_ctx) {
        const uint8_t **in = (const uint8_t **) af->frame->extended_data;
        uint8_t **out = &is->audio_buf1;
        int out_count = wanted_nb_sample * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size = av_samples_get_buffer_size(NULL, is->audio_tgt.channels, out_count, is->audio_tgt.fmt, 0);
        int len2;
        if (out_size < 0) {
             av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
            return -1;
        }
        if (wanted_nb_sample != af->frame->nb_samples) {
            if (swr_set_compensation(is->swr_ctx, (wanted_nb_sample - af->frame->nb_samples) * is->audio_tgt.freq / af->frame->sample_rate,
                                     wanted_nb_sample * is->audio_tgt.freq / af->frame->sample_rate) < 0) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                return -1;
            }
        }
        av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
        if (!is->audio_buf1) {
            return AVERROR(ENOMEM);
        }
        len2 = swr_convert(is->swr_ctx, out, out_count, in, af->frame->nb_samples);
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
            return -1;
        }
        if (len2 == out_count) {
            av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too samll.\n");
            if (swr_init(is->swr_ctx) < 0) {
                swr_free(&is->swr_ctx);
            }
        }
        is->audio_buf = is->audio_buf1;
        resample_data_size = len2 * is->audio_tgt.channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        is->audio_buf = af->frame->data[0];
        resample_data_size = data_size;
    }
    audio_clock0 = is->audio_clock;
    
    /* update the audio clock with the pts */
    if (!isnan(af->pts)) {
        is->audio_clock = af->pts + (double) af->frame->nb_samples / af->frame->sample_rate;
    } else {
        is->audio_clock = NAN;
    }
    
    is->audio_clock_serial = af->serial;
#ifdef DEBUG
    {
//        static double last_clock;
//        printf("audio: delay=%0.3f clock=%0.3f clock0=%0.3f\n",
//               is->audio_clock - last_clock,
//               is->audio_clock, audio_clock0);
//        last_clock = is->audio_clock;
    }
#endif
    return resample_data_size;
}

#ifdef __APPLE__
/* prepare a new audio buffer */
void sdl_audio_callback(void *opaque, Uint8 *stream, int len) {
    VideoState *is = opaque;
    int audio_size, len1;
    audio_callback_time = av_gettime_relative();
    while (len > 0) {
        if (is->audio_buf_index >= is->audio_buf_size) {
            audio_size = audio_decoder_frame(is);
            if (audio_size < 0) {
                is->audio_buf = NULL;
                is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size * is->audio_tgt.frame_size;
            } else {
                if (is->show_mode != SHOW_MODE_VIDEO) {
                    update_sample_display(is, (int16_t *) is->audio_buf, audio_size);
                }
                is->audio_buf_size = audio_size;
            }
            is->audio_buf_index = 0;
        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len) {
            len1 = len;
        }
        if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME) {
            memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
        } else {
            memset(stream, 0, len1);
            if (!is->muted && is->audio_buf) {
                SDL_MixAudio(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1, is->audio_volume);
            }
        }
        len -= len1;
        stream += len1;
        is->audio_buf_index += len1;
    }
    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;
    /* Let's assume the audio driver that is used by SDL has two periods. */
    if (!isnan(is->audio_clock)) {
        set_clock_at(&is->audclk, is->audio_clock - (double) (2 * is->audio_hw_buf_size + is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec,
                     is->audio_clock_serial, audio_callback_time / 1000000.0);
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }
}

int audio_open(void *opaque, int64_t wanted_channel_layout, int wanted_nb_channels, int wanted_sample_rate, AudioParams *audio_hw_params) {
    SDL_AudioSpec wanted_spec, spec;
    const char *env;
    static const int next_nb_channels[] = { 0, 0, 1, 6, 2, 6, 4, 6 };
    static const int next_sample_rates[] = { 0, 44100, 48000, 96000, 192000 };
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;
    
    env = SDL_getenv("SDL_AUDIO_CHANNELS");
    if (env) {
        wanted_nb_channels = atoi(env);
        wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
    }
    if (!wanted_channel_layout || wanted_nb_channels != av_get_channel_layout_nb_channels(wanted_channel_layout)) {
        wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
        wanted_channel_layout &= ~AV_CH_LAYOUT_STEREO_DOWNMIX;
    }
    wanted_nb_channels = av_get_channel_layout_nb_channels(wanted_channel_layout);
    wanted_spec.channels = wanted_nb_channels;
    wanted_spec.freq = wanted_sample_rate;
    if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
        av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count.\n");
        return -1;
    }
    while (next_sample_rate_idx && next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq) {
        next_sample_rate_idx--;
    }
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.silence = 0;
    wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE, 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
    wanted_spec.callback = sdl_audio_callback;
    wanted_spec.userdata = opaque;
    while (SDL_OpenAudio(&wanted_spec, &spec) < 0) {
        av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n", wanted_spec.channels, wanted_spec.freq, SDL_GetError());
        wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)];
        if (!wanted_spec.channels) {
            wanted_spec.freq = next_sample_rates[FFMIN(7, wanted_nb_channels)];
            wanted_spec.channels = wanted_nb_channels;
            if (!wanted_spec.freq) {
                av_log(NULL, AV_LOG_ERROR, "No more combinations to try, audio open failed\n");
                return -1;
            }
        }
        wanted_channel_layout = av_get_default_channel_layout(wanted_spec.channels);
    }
    if (spec.format != AUDIO_S16SYS) {
        av_log(NULL, AV_LOG_ERROR, "SDL advised audio format %d is not supported!\n", spec.channels);
        return -1;
    }
    if (spec.channels != wanted_nb_channels) {
        wanted_channel_layout = av_get_default_channel_layout(spec.channels);
        if (!wanted_channel_layout) {
            av_log(NULL, AV_LOG_ERROR, "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }
    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    audio_hw_params->channel_layout = wanted_channel_layout;
    audio_hw_params->channels = spec.channels;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
        av_log(NULL, AV_LOG_ERROR, "av_sample_get_buffer_size failed\n");
        return -1;
    }
    return spec.size;
}
#endif

int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block, int *serial) {
    MyAVPacketList *pkt1;
    int ret;
    pthread_mutex_lock(&q->mutex);
    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }
        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt) {
                q->last_pkt = NULL;
            }
            q->nb_packets--;
            q->size -= pkt1->pkt.size + sizeof(*pkt1);
            *pkt = pkt1->pkt;
            if (serial) {
                *serial = pkt1->serial;
            }
            av_free(pkt1);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            pthread_cond_wait(&q->cond, &q->mutex);
        }
    }
    pthread_mutex_unlock(&q->mutex);
    return ret;
}

int packet_queue_put(PacketQueue *q, AVPacket *pkt) {
    int ret;
    pthread_mutex_lock(&q->mutex);
    ret = packet_queue_put_private(q, pkt);
    pthread_mutex_unlock(&q->mutex);
    if (pkt != &flush_pkt && ret < 0) {
        av_packet_unref(pkt);
    }
    return ret;
}

int packet_queue_put_nullpacket(PacketQueue *q, int stream_index) {
    AVPacket pkt1, *pkt = &pkt1;
    av_init_packet(pkt);
    pkt->data = NULL;
    pkt->size = 0;
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int got_frame = 0;
    do {
        int ret = -1;
        if (d->queue->abort_request) {
            return -1;
        }
        if (!d->packet_pending || d->queue->serial != d->pkt_serial) {
            AVPacket pkt;
            do {
                if (d->queue->nb_packets == 0) {
                    pthread_cond_signal(&d->empty_queue_cond);
                }
                if (packet_queue_get(d->queue, &pkt, 1, &d->pkt_serial) < 0) {
                    return -1;
                }
                if (pkt.data == flush_pkt.data) {
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts;
                    d->next_pts_tb = d->start_pts_tb;
                }
            } while (pkt.data == flush_pkt.data || d->queue->serial != d->pkt_serial);
            av_packet_unref(&d->pkt);
            d->pkt_temp = d->pkt = pkt;
            d->packet_pending = 1;
        }
        
        switch (d->avctx->codec_type) {
            case AVMEDIA_TYPE_VIDEO:
                ret = avcodec_decode_video2(d->avctx, frame, &got_frame, &d->pkt_temp);
                if (got_frame) {
                    frame->pts = frame->pkt_dts;
                }
                break;
                
            case AVMEDIA_TYPE_AUDIO:
                ret = avcodec_decode_audio4(d->avctx, frame, &got_frame, &d->pkt_temp);
                if (got_frame) {
                    AVRational tb = (AVRational) { 1, frame->sample_rate };
                    if (frame->pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(frame->pts, d->avctx->time_base, tb);
                    } else if (frame->pkt_pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(frame->pkt_pts, av_codec_get_pkt_timebase(d->avctx), tb);
                    } else if (d->next_pts != AV_NOPTS_VALUE) {
                        frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                    }
                    if (frame->pts != AV_NOPTS_VALUE) {
                        d->next_pts = frame->pts + frame->nb_samples;
                        d->next_pts_tb = tb;
                    }
                }
                break;
            case AVMEDIA_TYPE_SUBTITLE:
                ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, &d->pkt_temp);
                break;
            default:
                break;
        }
        if (ret < 0) {
            d->packet_pending = 0;
        } else {
            d->pkt_temp.dts = d->pkt_temp.pts = AV_NOPTS_VALUE;
            if (d->pkt_temp.data) {
                if (d->avctx->codec_type != AVMEDIA_TYPE_AUDIO) {
                    ret = d->pkt_temp.size;
                }
                d->pkt_temp.data += ret;
                d->pkt_temp.size -= ret;
                if (d->pkt_temp.size <= 0) {
                    d->packet_pending = 0;
                }
            } else {
                if (!got_frame) {
                    d->packet_pending = 0;
                    d->finished = d->pkt_serial;
                }
            }
        }
    } while (!got_frame && !d->finished);
    return got_frame;
}

void decoder_release(Decoder *d) {
    av_packet_unref(&d->pkt);
    avcodec_free_context(&d->avctx);
}

void decoder_abort(Decoder *d, FrameQueue *fq) {
    packet_queue_abort(d->queue);
    frame_queue_signal(fq);
    pthread_join(d->decoder_tid, NULL);
    d->decoder_tid = NULL;
    packet_queue_flush(d->queue);
}

int64_t get_valid_channel_layout(int64_t channel_layout, int channels) {
    if (channel_layout && av_get_channel_layout_nb_channels(channel_layout) == channels) {
        return channel_layout;
    } else {
        return 0;
    }
}

int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1, enum AVSampleFormat fmt2, int64_t channel_count2) {
    if (channel_count1 == 1 && channel_count2 == 1) {
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    } else {
        return channel_count1 != channel_count2 || fmt1 != fmt2;
    }
}

void* audio_thread(void *arg) {
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    Frame *af;
    int last_serial = -1;
    int64_t dec_channel_layout;
    int reconfigure;
    int got_frame;
    AVRational tb;
    int ret = 0;
    if (!frame) {
        return NULL;
    }
    do {
        if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0) {
            avfilter_graph_free(&is->agraph);
            av_frame_free(&frame);
            return NULL;
        }
        if (got_frame) {
            tb = (AVRational) { 1, frame->sample_rate };
            dec_channel_layout = get_valid_channel_layout(frame->channel_layout, av_frame_get_channels(frame));
            reconfigure = cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.channels, frame->format, av_frame_get_channels(frame));
            if (reconfigure) {
                char buf1[1024], buf2[1024];
                av_get_channel_layout_string(buf1, sizeof(buf1), -1, is->audio_filter_src.channel_layout);
                av_get_channel_layout_string(buf2, sizeof(buf2), -1, dec_channel_layout);
                av_log(NULL, AV_LOG_DEBUG,
                       "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                       is->audio_filter_src.freq, is->audio_filter_src.channels, av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                       frame->sample_rate, av_frame_get_channels(frame), av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);
                is->audio_filter_src.fmt = frame->format;
                is->audio_filter_src.channel_layout = dec_channel_layout;
                is->audio_filter_src.channels = av_frame_get_channels(frame);
                is->audio_filter_src.freq = frame->sample_rate;
                last_serial = is->auddec.pkt_serial;
                if ((ret = configure_audio_filters(is, NULL, 1)) < 0) {
                    avfilter_graph_free(&is->agraph);
                    av_frame_free(&frame);
                    return NULL;
                }
            }
            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0) {
                avfilter_graph_free(&is->agraph);
                av_frame_free(&frame);
                av_err2str(ret);
                return NULL;
            }
            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame, 0)) >= 0) {
                tb = is->out_audio_filter->inputs[0]->time_base;
                if (!(af = frame_queue_peek_writable(&is->sampq))) {
                    avfilter_graph_free(&is->agraph);
                    av_frame_free(&frame);
                    return NULL;
                }
                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
                af->pos = av_frame_get_pkt_pos(frame);
                af->serial = is->auddec.pkt_serial;
                af->duration = av_q2d((AVRational) { frame->nb_samples, frame->sample_rate });
                av_frame_move_ref(af->frame, frame);
                frame_queue_push(&is->sampq);
                if (is->audioq.serial != is->auddec.pkt_serial) {
                    break;
                }
            }
            if (ret == AVERROR_EOF) {
                is->auddec.finished = is->auddec.pkt_serial;
            }
        }
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);
    return NULL;
}

int get_video_frame(VideoState *is, AVFrame *frame) {
    int got_picture;
    if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0) {
        return got_picture;
    }
    if (got_picture) {
        double dpts = NAN;
        if (frame->pts != AV_NOPTS_VALUE) {
            dpts = av_q2d(is->video_st->time_base) * frame->pts;
        }
        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);
        is->viddec_width = frame->width;
        is->viddec_height = frame->height;
    }
    return got_picture;
}

int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters, AVFrame *frame) {
    int ret;
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE };
    char sws_flags_str[512] = "";
    char buffersrc_args[256] = "";
    AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
    AVCodecParameters *codecpar = is->video_st->codecpar;
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
    av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", "bicubic");
    if (strlen(sws_flags_str)) {
        sws_flags_str[strlen(sws_flags_str) - 1] = '\0';
    }
    graph->scale_sws_opts = av_strdup(sws_flags_str);
    snprintf(buffersrc_args, sizeof(buffersrc_args), "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
             frame->width, frame->height, frame->format, is->video_st->time_base.num, is->video_st->time_base.den,
             codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1));
    if (fr.num && fr.den) {
        av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num, fr.den);
    }
    if ((ret = avfilter_graph_create_filter(&filt_src, avfilter_get_by_name("buffer"), "ffplay_buffer", buffersrc_args, NULL, graph)) < 0) {
        av_err2str(ret);
        return ret;
    }
    if ((ret = avfilter_graph_create_filter(&filt_out, avfilter_get_by_name("buffersink"), "ffplay_buffersink", NULL, NULL, graph)) < 0) {
        av_err2str(ret);
        return ret;
    }
    if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts, AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0) {
        av_err2str(ret);
        return ret;
    }
    last_filter = filt_out;
    
    /* Note: this macro adds a filter before the lastly added filter, so the
     * processing order of the filters is in reverse */
    #define INSERT_FILT(name, arg) do {                                     \
        AVFilterContext *filt_ctx;                                          \
        ret = avfilter_graph_create_filter(&filt_ctx,                       \
                                        avfilter_get_by_name(name),         \
                                        "ffplay_" name, arg, NULL, graph);  \
        if (ret < 0) {                                                      \
            av_err2str(ret);                                                \
            return ret;                                                     \
        }                                                                   \
        ret = avfilter_link(filt_ctx, 0, last_filter, 0);                   \
        if (ret < 0) {                                                      \
            av_err2str(ret);                                                \
            return ret;                                                     \
        }                                                                   \
        last_filter = filt_ctx;                                             \
    } while (0)
    if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0) {
        return ret;
    }
    is->in_video_filter = filt_src;
    is->out_video_filter = filt_out;
    return ret;
}

int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial) {
    Frame *vp;
    if (!(vp = frame_queue_peek_writable(&is->pictq))) {
        return -1;
    }
    vp->sar = src_frame->sample_aspect_ratio;
    vp->uploaded = 0;
    
    if (!vp->bmp || !vp->allocated ||
        vp->width != src_frame->width ||
        vp->height != src_frame->height ||
        vp->format != src_frame->format) {
        SDL_Event event;
        vp->allocated = 0;
        vp->reallocated = 0;
        vp->width = src_frame->width;
        vp->height = src_frame->height;
        
        /* the allocation must be done in the main thread to avoid
         locking problems. */
        event.type = FF_ALLOC_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);
        
        /* wait until the picture is allocated */
        pthread_mutex_lock(&is->pictq.mutex);
        while (!vp->allocated && !is->videoq.abort_request) {
            pthread_cond_wait(&is->pictq.cond, &is->pictq.mutex);
        }
        /* if the queue is aborted, we have to pop the pending ALLOC event or wait for the allocation to complete */
        //TODO 2.0
        if (is->videoq.abort_request && SDL_PeepEvents(&event, 1, SDL_GETEVENT, FF_ALLOC_EVENT, FF_ALLOC_EVENT) != 1) {
            while (!vp->allocated && !is->abort_request) {
                pthread_cond_wait(&is->pictq.cond, &is->pictq.mutex);
            }
        }
        pthread_mutex_unlock(&is->pictq.mutex);
        if (is->videoq.abort_request) {
            return -1;
        }
    }
    /* if the frame is not skipped, then display it */
    if (vp->bmp) {
        vp->pts = pts;
        vp->duration = duration;
        vp->pos = pos;
        vp->serial = serial;
        
        av_frame_move_ref(vp->frame, src_frame);
        frame_queue_push(&is->pictq);
    }
    return 0;
}

void* video_thread(void *arg) {
    int ret;
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    double pts;
    double duration;
    AVRational tb = is->video_st->time_base;
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL);
    AVFilterGraph *graph = avfilter_graph_alloc();
    AVFilterContext *filt_out = NULL, *filt_in = NULL;
    int last_w = 0;
    int last_h = 0;
    enum AVPixelFormat last_format = -2;
    int last_serial = -1;
    int last_vfilter_idx = 0;
    if (!graph) {
        av_frame_free(&frame);
        return NULL;
    }
    if (!frame) {
        avfilter_graph_free(&graph);
        return NULL;
    }
    while(1) {
        ret = get_video_frame(is, frame);
        if (ret < 0) {
            avfilter_graph_free(&graph);
            av_frame_free(&frame);
            return 0;
        }
        if (!ret) {
            continue;
        }
        if (last_w != frame->width || last_h != frame->height
            || last_format != frame->format
            || last_serial != is->viddec.pkt_serial
            || last_vfilter_idx != is->vfilter_idx) {
            av_log(NULL, AV_LOG_DEBUG,
                   "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
                   last_w, last_h,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"), last_serial,
                   frame->width, frame->height,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"), is->viddec.pkt_serial);
            avfilter_graph_free(&graph);
            graph = avfilter_graph_alloc();
            if ((ret = configure_video_filters(graph, is, NULL, frame)) < 0) {
                SDL_Event event;
                event.type = FF_QUIT_EVENT;
                event.user.data1 = is;
                SDL_PushEvent(&event);
                avfilter_graph_free(&graph);
                av_frame_free(&frame);
                return NULL;
            }
            filt_in = is->in_video_filter;
            filt_out = is->out_video_filter;
            last_w = frame->width;
            last_h = frame->height;
            last_format = frame->format;
            last_serial = is->viddec.pkt_serial;
            last_vfilter_idx = is->vfilter_idx;
            frame_rate = filt_out->inputs[0]->frame_rate;
        }
        
        ret = av_buffersrc_add_frame(filt_in, frame);
        if (ret < 0) {
            av_err2str(ret);
            avfilter_graph_free(&graph);
            av_frame_free(&frame);
            return NULL;
        }
        while (ret >= 0) {
            is->frame_last_returned_time = av_gettime_relative() / 1000000.0;
            ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
            if (ret < 0) {
                if (ret == AVERROR_EOF) {
                    is->viddec.finished = is->viddec.pkt_serial;
                }
                ret = 0;
                break;
            }
            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0) {
                is->frame_last_filter_delay = 0;
            }
            tb = filt_out->inputs[0]->time_base;
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational) { frame_rate.den, frame_rate.num }) : 0);
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            ret = queue_picture(is, frame, pts, duration, av_frame_get_pkt_pos(frame), is->viddec.pkt_serial);
            av_frame_unref(frame);
        }
    }
    return NULL;
}

void decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, pthread_cond_t empty_queue_cond) {
    memset(d, 0, sizeof(Decoder));
    d->avctx = avctx;
    d->queue = queue;
    d->empty_queue_cond = empty_queue_cond;
    d->start_pts = AV_NOPTS_VALUE;
}

int decoder_start(Decoder *d, void* (*fn) (void*), void *arg) {
    packet_queue_start(d->queue);
    int result = pthread_create(&d->decoder_tid, NULL, fn, arg);
    if (result != 0) {
        av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %d\n", result);
        return AVERROR(ENOMEM);
    }
    return 0;
}

int stream_component_open(VideoState *is, int stream_index) {
    int ret;
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    AVCodec *codec;
    AVDictionary *opts = NULL;
    int sample_rate, nb_channels;
    int64_t channel_layout;
    int stream_lowres = lowres;
    if (stream_index < 0 || stream_index >= ic->nb_streams) {
        return -1;
    }
    avctx = avcodec_alloc_context3(NULL);
    if (!avctx) {
        return AVERROR(ENOMEM);
    }
    ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
    if (ret < 0) {
        av_err2str(ret);
        avcodec_free_context(&avctx);
    }
    
    av_codec_set_pkt_timebase(avctx, ic->streams[stream_index]->time_base);
    codec = avcodec_find_decoder(avctx->codec_id);
    if (!codec) {
        av_log(NULL, AV_LOG_WARNING, "No codec could be found with id %d\n", avctx->codec_id);
        avcodec_free_context(&avctx);
        return AVERROR(ENOMEM);
    }
    avctx->codec_id = codec->id;
    if (stream_lowres > av_codec_get_max_lowres(codec)) {
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
               av_codec_get_max_lowres(codec));
        stream_lowres = av_codec_get_max_lowres(codec);
    }
    av_codec_set_lowres(avctx, stream_lowres);
    
#if FF_API_EMU_EDGE
    if(stream_lowres) avctx->flags |= CODEC_FLAG_EMU_EDGE;
#endif
    if (fast) {
        avctx->flags |= AV_CODEC_FLAG2_FAST;
    }
#if FF_API_EMU_EDGE
    if (codec->capabilities & AV_CODEC_CAP_DR1) {
        avctx->flags |= CODEC_FLAG_EMU_EDGE;
    }
#endif
    av_dict_set(&opts, "threads", "auto", 0);
    if (stream_lowres) {
        av_dict_set_int(&opts, "lowers", stream_lowres, 0);
    }
    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO || avctx->codec_type == AVMEDIA_TYPE_AUDIO) {
        av_dict_set(&opts, "refcounted_frames", "1", 0);
    }
    if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
        avcodec_free_context(&avctx);
        av_err2str(ret);
        return ret;
    }
    is->eof = 0;
    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_AUDIO: {
            AVFilterLink *link;
            is->audio_filter_src.freq = avctx->sample_rate;
            is->audio_filter_src.channels = avctx->channels;
            is->audio_filter_src.channel_layout = avctx->channel_layout;
            is->audio_filter_src.fmt = avctx->sample_fmt;
            if ((ret = configure_audio_filters(is, NULL, 0)) < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
            link = is->out_audio_filter->inputs[0];
            sample_rate = link->sample_rate;
            channel_layout = link->channel_layout;
            nb_channels = link->channels;
            
#ifdef __APPLE__
            /* prepare audio output */
            if ((ret = audio_open(is, channel_layout, nb_channels, sample_rate, &is->audio_tgt)) < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
#endif
            is->audio_hw_buf_size = ret;
            is->audio_src = is->audio_tgt;
            is->audio_buf_size = 0;
            is->audio_buf_index = 0;
            
            /* init averaging filter */
            is->audio_diff_avg_coef = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
            is->audio_diff_avg_count = 0;
            
            /* since we do not have a precise anough audio fifo fullness,
             we correct audio sync only if larger than this threshold */
            is->audio_diff_threshold = (double) (is->audio_hw_buf_size) / is->audio_tgt.bytes_per_sec;
            is->audio_stream = stream_index;
            is->audio_st = ic->streams[stream_index];
            
            decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread);
            if ((is->ic->iformat->flags & (AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK)) && !is->ic->iformat->read_seek) {
                is->auddec.start_pts = is->audio_st->start_time;
                is->auddec.start_pts_tb = is->audio_st->time_base;
            }
            if ((ret = decoder_start(&is->auddec, audio_thread, is)) < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
#ifdef __APPLE__
            SDL_PauseAudio(0);
#endif
        }
            break;
        case AVMEDIA_TYPE_VIDEO:
            is->video_stream = stream_index;
            is->video_st = ic->streams[stream_index];
            is->viddec_width = avctx->width;
            is->viddec_height = avctx->height;
            decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread);
            if ((ret = decoder_start(&is->viddec, video_thread, is)) < 0) {
                avcodec_free_context(&avctx);
                return ret;
            }
            is->queue_attachments_req = 1;
            break;
            
        case AVMEDIA_TYPE_SUBTITLE:
            break;
        default:
            break;
    }
    return ret;
}

/* seek in the stream */
void stream_seek(VideoState *is, int64_t pos, int64_t rel, int seek_by_bytes) {
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (seek_by_bytes) {
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        }
        is->seek_req = 1;
        pthread_cond_signal(&is->continue_read_thread);
    }
}

/* pause or resume the video */
void stream_toggle_pause(VideoState *is) {
    if (is->paused) {
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_update;
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);
    }
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

void step_to_next_frame(VideoState *is) {
    /* if the stream is paused unpause it, then step */
    if (is->paused) {
        stream_toggle_pause(is);
    }
    is->step = 1;
}

void read_thread_failed(VideoState *is, AVFormatContext *ic, pthread_mutex_t wait_mutex) {
    if (ic && !is->ic) {
        avformat_close_input(&ic);
    }
    SDL_Event event;
    event.type = FF_QUIT_EVENT;
    event.user.data1 = is;
    SDL_PushEvent(&event);
    pthread_mutex_destroy(&wait_mutex);
}

void* read_thread(void *arg) {
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int ret;
    int st_index[AVMEDIA_TYPE_NB];
    AVPacket pkt1, *pkt = &pkt1;
    int64_t stream_start_time;
    int pkt_in_play_range = 0;
    pthread_mutex_t wait_mutex;
    ret = pthread_mutex_init(&wait_mutex, NULL);
    int64_t pkt_ts;
    if (ret != 0) {
        av_log(NULL, AV_LOG_FATAL, "Read Thread SDL_CreateMutex(): %d\n", ret);
        read_thread_failed(is, NULL, wait_mutex);
        return NULL;
    }
    memset(st_index, -1, sizeof(st_index));
    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;
    is->eof = 0;
    
    ic = avformat_alloc_context();
    if (!ic) {
        av_log(NULL, AV_LOG_FATAL, "Can't allocate context.\n");
        read_thread_failed(is, ic, wait_mutex);
        return NULL;
    }
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;
    ret = avformat_open_input(&ic, is->filename, NULL, NULL);
    if (ret < 0) {
        read_thread_failed(is, ic, wait_mutex);
        av_err2str(ret);
        return NULL;
    }
    is->ic = ic;
    av_format_inject_global_side_data(ic);
    
    if ((ret = avformat_find_stream_info(ic, NULL)) < 0) {
        read_thread_failed(is, ic, wait_mutex);
        av_err2str(ret);
        return NULL;
    }
    if (ic->pb) {
        ic->pb->eof_reached = 0;
    }
    
    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;
    
    /* 如果start_time != AV_NOPTS_VALUE,则从start_time位置开始播放 */
    if (start_time != AV_NOPTS_VALUE) {
        int64_t timestamp = start_time;
        
        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE) {
            timestamp += ic->start_time;
        }
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_err2str(ret);
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                   is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }
    
    av_dump_format(ic, 0, is->filename, 0);
    
    for (int i = 0; i < ic->nb_streams; i++) {
        AVStream *st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        st->discard = AVDISCARD_ALL;
        if (wanted_stream_spec[type] && st_index[i] == -1) {
            if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0) {
                st_index[type] = i;
            }
        }
        
    }
    for (int i = 0; i < AVMEDIA_TYPE_NB; i++) {
        if (wanted_stream_spec[i] && st_index[i] == -1) {
            av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n", wanted_stream_spec[i], av_get_media_type_string(i));
            st_index[i] = INT_MAX;
        }
    }
    st_index[AVMEDIA_TYPE_VIDEO] = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
    st_index[AVMEDIA_TYPE_AUDIO] = av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO, st_index[AVMEDIA_TYPE_AUDIO],
                                                       st_index[AVMEDIA_TYPE_VIDEO], NULL, 0);
    st_index[AVMEDIA_TYPE_SUBTITLE] = av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE, st_index[AVMEDIA_TYPE_SUBTITLE],
                                                          (st_index[AVMEDIA_TYPE_AUDIO] > 0 ? st_index[AVMEDIA_TYPE_AUDIO] : st_index[AVMEDIA_TYPE_VIDEO]),
                                                           NULL, 0);
    
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
        AVCodecParameters *codecpar = st->codecpar;
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL);
        if (codecpar->width) {
#ifdef __APPLE__
            set_default_window_size(codecpar->width, codecpar->height, sar);
#endif
        }
    }
    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }
    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    if (is->show_mode == SHOW_MODE_NONE) {
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;
    }
    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }
    if (is->video_stream < 0 && is->audio_stream < 0) {
        read_thread_failed(is, ic, wait_mutex);
        return NULL;
    }
    for (; ;) {
        if (is->abort_request) {
            break;
        }
        if (is->paused != is->last_paused) {
            is->last_paused = is->paused;
            if (is->paused) {
                is->read_pause_return = av_read_pause(ic);
            } else {
                av_read_play(ic);
            }
        }
        if (is->seek_req) {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min = is->seek_rel > 0 ? seek_target - is->seek_rel + 2 : INT64_MIN;
            int64_t seek_max = is->seek_rel < 0 ? seek_target - is->seek_rel - 2 : INT64_MAX;
            
            // FIXME the +-2 is due to rounding being not done in the correct direction in generation
            //      of the seek_pos/seek_rel variables
            ret = avformat_seek_file(ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "%s: error while seeking\n", is->filename);
            } else {
                if (is->audio_stream >= 0) {
                    packet_queue_flush(&is->audioq);
                    packet_queue_put(&is->audioq, &flush_pkt);
                }
                if (is->video_stream >= 0) {
                    packet_queue_flush(&is->videoq);
                    packet_queue_put(&is->videoq, &flush_pkt);
                }
                if (is->subtitle_stream >= 0) {
                    packet_queue_flush(&is->subtitleq);
                    packet_queue_put(&is->subtitleq, &flush_pkt);
                }
                if (is->seek_flags & AVSEEK_FLAG_BYTE) {
                    set_clock(&is->extclk, NAN, 0);
                } else {
                    set_clock(&is->extclk, seek_target / (double) AV_TIME_BASE, 0);
                }
            }
            is->seek_req = 0;
            is->queue_attachments_req = 1;
            is->eof = 0;
            if (is->paused) {
                step_to_next_frame(is);
            }
        }
        if (is->queue_attachments_req) {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC) {
                AVPacket copy;
                if ((ret = av_copy_packet(&copy, &is->video_st->attached_pic)) < 0) {
                    read_thread_failed(is, ic, wait_mutex);
                }
                packet_queue_put(&is->videoq, &copy);
                packet_queue_put_nullpacket(&is->videoq, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }
        
        /* if the queue are full, no need to read more */
        if ((is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE ||
             ((is->audioq.nb_packets > MIN_FRAMES || is->audio_stream < 0 || is->audioq.abort_request) &&
             (is->videoq.nb_packets > MIN_FRAMES || is->video_stream < 0 || is->videoq.abort_request ||
             (is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) &&
             (is->subtitleq.nb_packets > MIN_FRAMES || is->subtitle_stream < 0 || is->subtitleq.abort_request)))) {
            /* wait 10 ms */
            pthread_mutex_lock(&wait_mutex);
//            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            pthread_mutex_unlock(&wait_mutex);
            continue;
        }
        if (!is->paused &&
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && frame_queue_nb_remaining(&is->sampq) == 0)) &&
            (!is->video_st || (is->viddec.finished == is->videoq.serial && frame_queue_nb_remaining(&is->pictq) == 0))) {
        }
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
                if (is->video_stream >= 0) {
                    packet_queue_put_nullpacket(&is->videoq, is->video_stream);
                }
                if (is->audio_stream >= 0) {
                    packet_queue_put_nullpacket(&is->audioq, is->audio_stream);
                }
                if (is->subtitle_stream >= 0) {
                    packet_queue_put_nullpacket(&is->subtitleq, is->subtitle_stream);
                }
                is->eof = 1;
            }
            if (ic->pb && ic->pb->error) {
                break;
            }
            pthread_mutex_lock(&wait_mutex);
//            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            pthread_mutex_unlock(&wait_mutex);
            continue;
        } else {
            is->eof = 0;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        stream_start_time = ic->streams[pkt->stream_index]->start_time;
        pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        pkt_in_play_range = duration == AV_NOPTS_VALUE || (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
                            av_q2d(ic->streams[pkt->stream_index]->time_base) -
                            (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000 <= ((double) duration / 1000000);
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            av_packet_unref(pkt);
        }
    }
    return 0;
}


static void stream_component_close(VideoState *is, int stream_index) {
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    
    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    avctx = ic->streams[stream_index]->codec;
    
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            decoder_abort(&is->auddec, &is->sampq);
            SDL_CloseAudio();
            decoder_release(&is->auddec);
            swr_free(&is->swr_ctx);
            av_freep(&is->audio_buf1);
            is->audio_buf1_size = 0;
            is->audio_buf = NULL;
            
            if (is->rdft) {
                av_rdft_end(is->rdft);
                av_freep(&is->rdft_data);
                is->rdft = NULL;
                is->rdft_bits = 0;
            }
            break;
        case AVMEDIA_TYPE_VIDEO:
            decoder_abort(&is->viddec, &is->pictq);
            decoder_release(&is->viddec);
            break;
        default:
            break;
    }
    
    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    avcodec_close(avctx);
    switch (avctx->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            is->audio_st = NULL;
            is->audio_stream = -1;
            break;
        case AVMEDIA_TYPE_VIDEO:
            is->video_st = NULL;
            is->video_stream = -1;
            break;
        case AVMEDIA_TYPE_SUBTITLE:
            is->subtitle_st = NULL;
            is->subtitle_stream = -1;
            break;
        default:
            break;
    }
}

void stream_close(VideoState *is) {
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    is->abort_request = 1;
    pthread_join(is->read_tid, NULL);
    
    /* close each stream */
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);
    
    avformat_close_input(&is->ic);
    
    packet_queue_destroy(&is->videoq);
    packet_queue_destroy(&is->audioq);
    packet_queue_destroy(&is->subtitleq);
    
    /* free all pictures */
    frame_queue_destory(&is->pictq);
    frame_queue_destory(&is->sampq);
    frame_queue_destory(&is->subpq);
    pthread_cond_destroy(&is->continue_read_thread);
    sws_freeContext(is->img_convert_ctx);
    sws_freeContext(is->sub_convert_ctx);
    av_free(is->filename);
    if (is->vis_texture)
        SDL_DestroyTexture(is->vis_texture);
    if (is->sub_texture)
        SDL_DestroyTexture(is->sub_texture);
    av_free(is);
}

void do_exit(VideoState *is) {
    if (is) {
        stream_close(is);
    }
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (window)
        SDL_DestroyWindow(window);
    avformat_network_deinit();
    SDL_Quit();
    av_log(NULL, AV_LOG_QUIET, "%s", "");
}

VideoState *stream_open(const char *filename);

VideoState *stream_open(const char *filename) {
    VideoState *is = av_mallocz(sizeof(VideoState));
    if (!is) {
        return NULL;
    }
    is->filename = av_strdup(filename);
    if (!is->filename) {
        stream_close(is);
        return NULL;
    }
    is->ytop = 0;
    is->xleft = 0;
    if (frame_queue_init(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0) {
        stream_close(is);
        return NULL;
    }
    if (frame_queue_init(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0) {
        stream_close(is);
        return NULL;
    }
    if (frame_queue_init(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0) {
        stream_close(is);
        return NULL;
    }
    if (packet_queue_init(&is->videoq) < 0 ||
        packet_queue_init(&is->audioq) < 0 ||
        packet_queue_init(&is->subtitleq) < 0) {
        stream_close(is);
        return NULL;
    }
    pthread_cond_init(&is->continue_read_thread, NULL);
    init_clock(&is->vidclk, &is->videoq.serial);
    init_clock(&is->audclk, &is->audioq.serial);
    init_clock(&is->extclk, &is->extclk.serial);
    is->audio_clock_serial = -1;
    is->audio_volume = SDL_MIX_MAXVOLUME;
    is->muted = 0;
    is->av_sync_type = av_sync_type;
    int result = pthread_create(&is->read_tid, NULL, read_thread, is);
    if (result != 0) {
        stream_close(is);
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %d\n", result);
        return NULL;
    }
    return is;
}

void check_external_clock_speed(VideoState *is) {
    if ((is->audio_stream >= 0 && is->audioq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES) ||
        (is->video_stream >= 0 && is->videoq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES)) {
        set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN, is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP));
    } else if ((is->video_stream < 0 || is->videoq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES) &&
               (is->audio_stream < 0 || is->audioq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES)) {
        set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX, is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP));
    } else {
        double speed = is->extclk.speed;
        if (speed != 1.0) {
            set_clock_speed(&is->extclk, speed + EXTERNAL_CLOCK_SPEED_STEP * (1.0 - speed) / fabs(1.0 - speed));
        }
    }
}
//
//int video_open(VideoState *is, Frame *vp) {
//    int w, h;
//    if (vp && vp->width) {
//        set_default_window_size(vp->width, vp->height, vp->sar);
//    }
//    if (screen_width) {
//        w = screen_width;
//        h = screen_height;
//    } else {
//        w = default_width;
//        h = default_height;
//    }
//    if (!window) {
//        int flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
//        if (is_full_screen) {
//            flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
//        }
//        window = SDL_CreateWindow("", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
//        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
//        if (window) {
//            SDL_RendererInfo info;
//            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
//            if (renderer) {
//                if (!SDL_GetRendererInfo(renderer, &info)) {
//                    av_log(NULL, AV_LOG_VERBOSE, "Inilialized %s renderer.\n", info.name);
//                }
//            }
//        }
//    } else {
//        SDL_SetWindowSize(window, w, h);
//    }
//    if (!window && !renderer) {
//        av_log(NULL, AV_LOG_ERROR, "SDL: could not set video model - exiting\n");
//        return AVERROR(ENOMEM);
//    }
//    is->width = w;
//    is->height = h;
//    return 0;
//}
//
//int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture) {
//    Uint32 format;
//    int access, w, h;
//    if (SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 || new_width != w || new_height != h || new_format != format) {
//        void *pixels;
//        int picth;
//        SDL_DestroyTexture(*texture);
//        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height))) {
//            av_log(NULL, AV_LOG_ERROR, "SDL_CreateTexture(): %s\n", SDL_GetError());
//            return -1;
//        }
//        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0) {
//            return -1;
//        }
//        if (init_texture) {
//            if (SDL_LockTexture(*texture, NULL, &pixels, &picth) < 0) {
//                return -1;
//            }
//            memset(pixels, 0, picth * new_height);
//            SDL_UnlockTexture(*texture);
//        }
//    }
//    return 0;
//}
//
//int upload_texture(SDL_Texture *tex, AVFrame *frame, struct SwsContext **img_convert_ctx) {
//    int ret = 0;
//    switch (frame->format) {
//        case AV_PIX_FMT_YUV420P:
//            ret = SDL_UpdateYUVTexture(tex, NULL, frame->data[0], frame->linesize[0],
//                                       frame->data[1], frame->linesize[1],
//                                       frame->data[2], frame->linesize[2]);
//            break;
//            
//        case AV_PIX_FMT_BGRA:
//            ret = SDL_UpdateTexture(tex, NULL, frame->data[0], frame->linesize[0]);
//            break;
//        default:
//            /* This should only happed if we are not using avfilter */
//            *img_convert_ctx = sws_getCachedContext(*img_convert_ctx,
//                                                    frame->width, frame->height, frame->format, frame->width, frame->height,
//                                                    AV_PIX_FMT_BGRA, SWS_BICUBIC, NULL, NULL, NULL);
//            if (*img_convert_ctx != NULL) {
//                uint8_t *pixels;
//                int pitch;
//                if (!SDL_LockTexture(tex, NULL, (void **) &pixels, &pitch)) {
//                    sws_scale(*img_convert_ctx, (const uint8_t * const *) frame->data, frame->linesize,
//                              0, frame->height, &pixels, &pitch);
//                    SDL_UnlockTexture(tex);
//                }
//            } else {
//                av_log(NULL, AV_LOG_FATAL, "Can't initialize the conversion context.\n");
//                ret = -1;
//            }
//            break;
//    }
//    return ret;
//}
//
//extern inline int compute_mod(int a, int b);
//
//inline int compute_mod(int a, int b) {
//    return a < 0 ? a % b + b : a % b;
//}
//
//extern inline void fill_rectangle(int x, int y, int w, int h);
//
//inline void fill_rectangle(int x, int y, int w, int h) {
//    SDL_Rect rect;
//    rect.x = x;
//    rect.y = y;
//    rect.w = w;
//    rect.h = h;
//    if (w && h) {
//        SDL_RenderFillRect(renderer, &rect);
//    }
//}

//void video_image_display(VideoState *is) {
//    Frame *vp;
//    Frame *sp = NULL;
//    SDL_Rect rect;
//    vp = frame_queue_peek_last(&is->pictq);
//    if (vp->bmp) {
//        if (is->subtitle_st) {
//            if (frame_queue_nb_remaining(&is->subpq) > 0) {
//                sp = frame_queue_peek(&is->subpq);
//                if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
//                    if (!sp->uploaded) {
//                        uint8_t *pixels;
//                        int picth;
//                        if (!sp->width || !sp->height) {
//                            sp->width = vp->width;
//                            sp->height = vp->height;
//                        }
//                        if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0) {
//                            return;
//                        }
//                        for (int i = 0; i < sp->sub.num_rects; i++) {
//                            AVSubtitleRect *sub_rect = sp->sub.rects[i];
//                            sub_rect->x = av_clip(sub_rect->x, 0, sp->width);
//                            sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
//                            sub_rect->w = av_clip(sub_rect->w, 0, sp->width - sub_rect->x);
//                            sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);
//                            
//                            is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx, sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
//                                                                       sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA, 0, NULL, NULL, NULL);
//                            if (!is->sub_convert_ctx) {
//                                av_log(NULL, AV_LOG_FATAL, "Can't initialize the conversion context.\n");
//                                return;
//                            }
//                            if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *) sub_rect, (void **)&pixels, &picth)) {
//                                sws_scale(is->sub_convert_ctx, (const uint8_t * const *) sub_rect->data, sub_rect->linesize, 0, sub_rect->h, &pixels, &picth);
//                                SDL_UnlockTexture(is->sub_texture);
//                            }
//                        }
//                        sp->uploaded = 1;
//                    }
//                } else {
//                    sp = NULL;
//                }
//            }
//        }
//        
//        calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);
//        
//        if (!vp->uploaded) {
//            if (upload_texture(vp->bmp, vp->frame, &is->img_convert_ctx) < 0) {
//                return;
//            }
//            vp->uploaded = 1;
//        }
//        SDL_RenderCopy(renderer, vp->bmp, NULL, &rect);
//        if (sp) {
//#if USE_ONEPASS_SUBTITLE_RENDER
//            SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
//#else
//            double xratio = (double) rect.w / (double) sp->width;
//            double yratio = (double) rect.h / (double) sp->height;
//            for (int i = 0; i < sp->sub.num_rects; i++) {
//                SDL_Rect *sub_rect = (SDL_Rect *) sp->sub.rects[i];
//                SDL_Rect target = {.x = rect.y + sub_rect->x * xratio,
//                                   .y = rect.y + sub_rect->y * yratio,
//                                   .w = sub_rect->w * xratio,
//                                   .h = sub_rect->h * yratio};
//                SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target);
//            }
//#endif
//        }
//    }
//}
//
//void video_display(VideoState *is) {
//    if (!window) {
//        int result = video_open(is, NULL);
//        if (result < 0) {
//            //TODO not video open, exit player
//        }
//    }
//    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
//    SDL_RenderClear(renderer);
//    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO) {
////        video_audio_display(is);
//    } else if (is->video_st) {
//        video_image_display(is);
//    }
//    SDL_RenderPresent(renderer);
//}
//
//double vp_duration(VideoState *is, Frame *vp, Frame *nextvp) {
//    if (vp->serial == nextvp->serial) {
//        double duration = nextvp->pts - vp->pts;
//        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration) {
//            return vp->duration;
//        } else {
//            return duration;
//        }
//    } else {
//        return 0.0;
//    }
//}
//
//double compute_target_delay(double delay, VideoState *is) {
//    double sync_threshold, diff = 0;
//    
//    /* update delay to follow master synchronisation source */
//    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
//        /* if video is slave, we try to correct big delays by
//         duplicating or deleting a frame */
//        diff = get_clock(&is->vidclk) - get_master_clock(is);
//        
//        /* skip or repeat frame. We take into account the
//         delay to compute the threshold. I still don't know
//         if it is the best guess */
//        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX, delay));
//        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
//            if (diff <= -sync_threshold)
//                delay = FFMAX(0, delay + diff);
//            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD)
//                delay = delay + diff;
//            else if (diff >= sync_threshold)
//                delay = 2 * delay;
//        }
//    }
//    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n", delay, -diff);
//    return delay;
//}
//
//static void update_video_pts(VideoState *is, double pts, int64_t pos, int serial) {
//    /* update current video pts */
//    set_clock(&is->vidclk, pts, serial);
//    sync_clock_to_slave(&is->extclk, &is->vidclk);
//}
//
//void video_retry(VideoState *is, double *remaining_time) {
//    double time;
//    Frame *sp, *sp2;
//    if (frame_queue_nb_remaining(&is->pictq) == 0) {
//        //nothing to do, no picture to display in the queue
//    } else {
//        double last_duration, delay;
//        Frame *vp, *lastvp;
//        
//        /* dequeue the picture */
//        lastvp = frame_queue_peek_last(&is->pictq);
//        vp = frame_queue_peek(&is->pictq);
//        
//        if (vp->serial != is->videoq.serial) {
//            frame_queue_next(&is->pictq);
//            //TODO
//            video_retry(is, remaining_time);
//            return;
//        }
//        if (lastvp->serial != vp->serial) {
//            is->frame_timer = av_gettime_relative() / 1000000.0;
//        }
//        if (is->paused) {
//            /* display picture*/
//            if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
//                video_display(is);
//                return;
//            }
//        }
//        /* compute nominal last_duration */
//        last_duration = vp_duration(is, lastvp, vp);
//        delay = compute_target_delay(last_duration, is);
//        
//        time = av_gettime_relative() / 1000000.0;
//        if (time < is->frame_timer + delay) {
//            *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
//            /* display picture*/
//            if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
//                video_display(is);
//                return;
//            }
//        }
//        is->frame_timer += delay;
//        if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX) {
//            is->frame_timer = time;
//        }
//        pthread_mutex_lock(&is->pictq.mutex);
//        if (!isnan(vp->pts)) {
//            update_video_pts(is, vp->pts, vp->pos, vp->serial);
//        }
//        pthread_mutex_unlock(&is->pictq.mutex);
//
//        if (is->subtitle_st) {
//            while (frame_queue_nb_remaining(&is->subpq) > 0) {
//                sp = frame_queue_peek(&is->subpq);
//                if (frame_queue_nb_remaining(&is->subpq) > 1) {
//                    sp2 = frame_queue_peek_next(&is->subpq);
//                } else {
//                    sp2 = NULL;
//                }
//                if (sp->serial != is->subtitleq.serial ||
//                    (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000))) ||
//                    (sp2 && is->vidclk.pts > (sp2->pts + ((float)sp2->sub.start_display_time / 1000)))) {
//                    if (sp->uploaded) {
//                        for (int i = 0; i < sp->sub.num_rects; i++) {
//                            AVSubtitleRect *sub_rect = sp->sub.rects[i];
//                            uint8_t *pixels;
//                            int pitch;
//                            if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
//                                for (int j = 0; j < sub_rect->h; j++, pixels += pitch) {
//                                    memset(pixels, 0, sub_rect->w << 2);
//                                }
//                                SDL_UnlockTexture(is->sub_texture);
//                            }
//                        }
//                    }
//                    frame_queue_next(&is->subpq);
//                } else {
//                    break;
//                }
//            }
//        }
//        frame_queue_next(&is->pictq);
//        is->force_refresh = 1;
//        if (is->step && !is->paused) {
//            stream_toggle_pause(is);
//        }
//    }
//    if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown) {
//        video_display(is);
//    }
//}
//
///* called to display each frame */
//static void video_refresh(void *opaque, double *remaining_time)
//{
//    VideoState *is = opaque;
//    double time;
//    
//    Frame *sp, *sp2;
//    
//    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK)
//        check_external_clock_speed(is);
//    
//    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
//        time = av_gettime_relative() / 1000000.0;
//        if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
//            video_display(is);
//            is->last_vis_time = time;
//        }
//        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
//    }
//    
//    if (is->video_st) {
//    retry:
//        if (frame_queue_nb_remaining(&is->pictq) == 0) {
//            // nothing to do, no picture to display in the queue
//        } else {
//            double last_duration, delay;
//            Frame *vp, *lastvp;
//            
//            /* dequeue the picture */
//            lastvp = frame_queue_peek_last(&is->pictq);
//            vp = frame_queue_peek(&is->pictq);
//            
//            if (vp->serial != is->videoq.serial) {
//                frame_queue_next(&is->pictq);
//                goto retry;
//            }
//            
//            if (lastvp->serial != vp->serial)
//                is->frame_timer = av_gettime_relative() / 1000000.0;
//            
//            if (is->paused)
//                goto display;
//            
//            /* compute nominal last_duration */
//            last_duration = vp_duration(is, lastvp, vp);
//            delay = compute_target_delay(last_duration, is);
//            
//            time= av_gettime_relative()/1000000.0;
//            if (time < is->frame_timer + delay) {
//                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
//                goto display;
//            }
//            
//            is->frame_timer += delay;
//            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
//                is->frame_timer = time;
//            
//            pthread_mutex_lock(&is->pictq.mutex);
//            if (!isnan(vp->pts))
//                update_video_pts(is, vp->pts, vp->pos, vp->serial);
//            pthread_mutex_unlock(&is->pictq.mutex);
//            
//            if (is->subtitle_st) {
//                while (frame_queue_nb_remaining(&is->subpq) > 0) {
//                    sp = frame_queue_peek(&is->subpq);
//                    
//                    if (frame_queue_nb_remaining(&is->subpq) > 1)
//                        sp2 = frame_queue_peek_next(&is->subpq);
//                    else
//                        sp2 = NULL;
//                    
//                    if (sp->serial != is->subtitleq.serial
//                        || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
//                        || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
//                    {
//                        if (sp->uploaded) {
//                            int i;
//                            for (i = 0; i < sp->sub.num_rects; i++) {
//                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
//                                uint8_t *pixels;
//                                int pitch, j;
//                                
//                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
//                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
//                                        memset(pixels, 0, sub_rect->w << 2);
//                                    SDL_UnlockTexture(is->sub_texture);
//                                }
//                            }
//                        }
//                        frame_queue_next(&is->subpq);
//                    } else {
//                        break;
//                    }
//                }
//            }
//            
//            frame_queue_next(&is->pictq);
//            is->force_refresh = 1;
//            
//            if (is->step && !is->paused)
//                stream_toggle_pause(is);
//        }
//    display:
//        /* display picture */
//        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
//            video_display(is);
//    }
//    is->force_refresh = 0;
//}
//
//void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {
//    double remaining_time = 0.0;
//    SDL_PumpEvents();
//    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
//        if (remaining_time > 0.0) {
//            av_usleep(remaining_time * 1000000.0);
//        }
//        remaining_time = REFRESH_RATE;
//        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh)) {
//            video_refresh(is, &remaining_time);
//        }
//        SDL_PumpEvents();
//    }
//}
//
//void alloc_picture(VideoState *is) {
//    Frame *vp;
//    int sdl_format;
//    vp = &is->pictq.queue[is->pictq.windex];
//    video_open(is, vp);
//    
//    if (vp->format == AV_PIX_FMT_YUV420P) {
//        sdl_format = SDL_PIXELFORMAT_YV12;
//    } else {
//        sdl_format = SDL_PIXELFORMAT_ARGB8888;
//    }
//    
//    if (realloc_texture(&vp->bmp, sdl_format, vp->width, vp->height, SDL_BLENDMODE_NONE, 0) < 0) {
//        //TODO exit application
//        /* SDL allocates a buffer smaller than requested if the video
//         * overlay hardware is unable to support the requested size. */
//        av_log(NULL, AV_LOG_FATAL,
//               "Error: the video system does not support an image\n"
//               "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
//               "to reduce the image size.\n", vp->width, vp->height );
//
//    }
//    pthread_mutex_lock(&is->pictq.mutex);
//    vp->allocated = 1;
//    pthread_cond_signal(&is->pictq.cond);
//    pthread_mutex_unlock(&is->pictq.mutex);
//}
//
//void event_loop(VideoState *cur_stream) {
//    SDL_Event event;
//    for (;;) {
//        refresh_loop_wait_event(cur_stream, &event);
//        switch (event.type) {
//            case SDL_KEYDOWN:
//                switch (event.key.keysym.sym) {
//                        
//                    default:
//                        break;
//                }
//                break;
//                
//            case SDLK_q: 
//                do_exit(cur_stream);
//                break;
//            case SDLK_SPACE:
//                stream_toggle_pause(cur_stream);
//                cur_stream->step = 0;
//                break;
//
//            case FF_ALLOC_EVENT:
//                alloc_picture(event.user.data1);
//                break;
//            default:
//                break;
//        }
//        
//    }
//}

VideoState* init_ffplay(const char *filename) {
    
#ifdef __APPLE__
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER);
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);
#endif

    
    av_init_packet(&flush_pkt);
    flush_pkt.data = (uint8_t *)&flush_pkt;
    VideoState *is = stream_open(filename);

//#ifdef __APPLE__
//    event_loop(is);
//#endif
    return is;
}
