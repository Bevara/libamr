/*
 *			GPAC - Multimedia Framework C SDK
 *
 *  This file is part of GPAC / AMR decoder filter, based on opencore-amr
 *  (https://sourceforge.net/projects/opencore-amr/), covering both the
 *  narrowband (8 kHz, the codec of GSM voice calls) and wideband (16 kHz)
 *  variants.
 *
 *  The .amr storage format is a magic string followed by frames whose first
 *  byte carries the coding mode; the frame lengths below come from
 *  3GPP TS 26.101 (NB) and TS 26.201 (WB).
 */

#include <gpac/filters.h>
#include <gpac/constants.h>
#include <string.h>
#include <stdlib.h>

#include <opencore-amrnb/interf_dec.h>
#include <opencore-amrwb/dec_if.h>

#define AMRNB_MAGIC "#!AMR\n"
#define AMRWB_MAGIC "#!AMR-WB\n"
#define AMRNB_FRAME_SAMPLES 160 /* 20 ms at 8 kHz */
#define AMRWB_FRAME_SAMPLES 320 /* 20 ms at 16 kHz */

/* Bytes per frame including the mode byte, indexed by mode. */
static const int amrnb_frame_sizes[16] = {13, 14, 16, 18, 20, 21, 27, 32, 6, 0, 0, 0, 0, 0, 0, 1};
static const int amrwb_frame_sizes[16] = {18, 24, 33, 37, 41, 47, 51, 59, 61, 6, 0, 0, 0, 0, 1, 1};

typedef struct
{
	GF_FilterPid *ipid, *opid;
	Bool is_playing;
} GF_AMRDecCtx;

static GF_Err amrdec_configure_pid(GF_Filter *filter, GF_FilterPid *pid, Bool is_remove)
{
	GF_AMRDecCtx *ctx = (GF_AMRDecCtx *)gf_filter_get_udta(filter);

	if (is_remove)
	{
		if (ctx->opid)
		{
			gf_filter_pid_remove(ctx->opid);
			ctx->opid = NULL;
		}
		ctx->ipid = NULL;
		return GF_OK;
	}
	if (!gf_filter_pid_check_caps(pid))
		return GF_NOT_SUPPORTED;

	ctx->ipid = pid;
	gf_filter_pid_set_framing_mode(pid, GF_TRUE);

	if (!ctx->opid)
		ctx->opid = gf_filter_pid_new(filter);

	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_STREAM_TYPE, &PROP_UINT(GF_STREAM_AUDIO));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CODECID, &PROP_UINT(GF_CODECID_RAW));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_AUDIO_FORMAT, &PROP_UINT(GF_AUDIO_FMT_S16));
	/* Narrowband is the common case; process() sets 16000 for a wideband file. */
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_SAMPLE_RATE, &PROP_UINT(8000));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_TIMESCALE, &PROP_UINT(8000));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_NUM_CHANNELS, &PROP_UINT(1));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CHANNEL_LAYOUT, &PROP_LONGUINT(GF_AUDIO_CH_FRONT_CENTER));

	return GF_OK;
}

static Bool amrdec_process_event(GF_Filter *filter, const GF_FilterEvent *evt)
{
	GF_AMRDecCtx *ctx = (GF_AMRDecCtx *)gf_filter_get_udta(filter);
	switch (evt->base.type)
	{
	case GF_FEVT_PLAY:
		ctx->is_playing = GF_TRUE;
		return GF_FALSE;
	case GF_FEVT_STOP:
		ctx->is_playing = GF_FALSE;
		return GF_FALSE;
	default:
		return GF_FALSE;
	}
}

static GF_Err amrdec_process(GF_Filter *filter)
{
	GF_FilterPacket *pck, *dst_pck;
	u8 *data, *output;
	u32 size, pos, sample_rate, frame_samples, nb_frames = 0, out_samples = 0;
	Bool wideband;
	void *state;
	s16 *pcm = NULL;
	GF_AMRDecCtx *ctx = (GF_AMRDecCtx *)gf_filter_get_udta(filter);

	pck = gf_filter_pid_get_packet(ctx->ipid);
	if (!pck)
	{
		if (gf_filter_pid_is_eos(ctx->ipid))
		{
			gf_filter_pid_set_eos(ctx->opid);
			return GF_EOS;
		}
		return GF_OK;
	}
	data = (u8 *)gf_filter_pck_get_data(pck, &size);
	if (!data)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_IO_ERR;
	}

	if ((size > strlen(AMRWB_MAGIC)) && !memcmp(data, AMRWB_MAGIC, strlen(AMRWB_MAGIC)))
	{
		wideband = GF_TRUE;
		pos = (u32)strlen(AMRWB_MAGIC);
		sample_rate = 16000;
		frame_samples = AMRWB_FRAME_SAMPLES;
	}
	else if ((size > strlen(AMRNB_MAGIC)) && !memcmp(data, AMRNB_MAGIC, strlen(AMRNB_MAGIC)))
	{
		wideband = GF_FALSE;
		pos = (u32)strlen(AMRNB_MAGIC);
		sample_rate = 8000;
		frame_samples = AMRNB_FRAME_SAMPLES;
	}
	else
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[AMRDec] Missing #!AMR header (multi-channel files are not supported)\n"));
		return GF_NON_COMPLIANT_BITSTREAM;
	}

	/* An upper bound on the frame count: the shortest frame is 6 bytes. */
	pcm = (s16 *)gf_malloc(((size_t)size / 6 + 1) * frame_samples * sizeof(s16));
	if (!pcm)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OUT_OF_MEM;
	}

	state = wideband ? D_IF_init() : Decoder_Interface_init();
	if (!state)
	{
		gf_free(pcm);
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OUT_OF_MEM;
	}

	while (pos < size)
	{
		u32 mode = (data[pos] >> 3) & 0x0F;
		int frame_size = wideband ? amrwb_frame_sizes[mode] : amrnb_frame_sizes[mode];
		if (!frame_size || (pos + (u32)frame_size > size))
			break;
		if (wideband)
			D_IF_decode(state, data + pos, pcm + out_samples, 0);
		else
			Decoder_Interface_Decode(state, data + pos, pcm + out_samples, 0);
		out_samples += frame_samples;
		nb_frames++;
		pos += (u32)frame_size;
	}

	if (wideband)
		D_IF_exit(state);
	else
		Decoder_Interface_exit(state);
	gf_filter_pid_drop_packet(ctx->ipid);

	if (!nb_frames)
	{
		gf_free(pcm);
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[AMRDec] No decodable frame found\n"));
		return GF_NON_COMPLIANT_BITSTREAM;
	}

	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_SAMPLE_RATE, &PROP_UINT(sample_rate));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_TIMESCALE, &PROP_UINT(sample_rate));

	dst_pck = gf_filter_pck_new_alloc(ctx->opid, out_samples * (u32)sizeof(s16), &output);
	if (!dst_pck)
	{
		gf_free(pcm);
		return GF_OUT_OF_MEM;
	}
	memcpy(output, pcm, out_samples * sizeof(s16));
	gf_free(pcm);

	gf_filter_pck_set_cts(dst_pck, 0);
	gf_filter_pck_set_sap(dst_pck, GF_FILTER_SAP_1);
	gf_filter_pck_set_duration(dst_pck, out_samples);
	gf_filter_pck_send(dst_pck);

	gf_filter_pid_set_eos(ctx->opid);
	return GF_EOS;
}

static void amrdec_finalize(GF_Filter *filter)
{
}

static const GF_FilterCapability AMRDecCaps[] =
	{
		CAP_UINT(GF_CAPS_INPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_FILE),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_FILE_EXT, "amr|awb"),
		CAP_STRING(GF_CAPS_INPUT, GF_PROP_PID_MIME, "audio/amr|audio/amr-wb|audio/3gpp"),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_AUDIO),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_CODECID, GF_CODECID_RAW),
};

GF_FilterRegister AMRDecoderRegister = {
	.name = "amrdec",
	GF_FS_SET_DESCRIPTION("AMR narrowband and wideband decoder")
		GF_FS_SET_HELP("This filter decodes AMR-NB and AMR-WB speech files using opencore-amr.")
			.private_size = sizeof(GF_AMRDecCtx),
	SETCAPS(AMRDecCaps),
	.configure_pid = amrdec_configure_pid,
	.process = amrdec_process,
	.process_event = amrdec_process_event,
	.finalize = amrdec_finalize,
};

const GF_FilterRegister *EMSCRIPTEN_KEEPALIVE amrdec_register(GF_FilterSession *session)
{
	return &AMRDecoderRegister;
}

#include "filter_register.h"
__attribute__((constructor))
void register_amrdec(void) {
    gf_filter_auto_register("amrdec", amrdec_register);
}
