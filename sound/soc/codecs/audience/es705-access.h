/*
 * es705-access.h  --  ES705 Soc Audio access values
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES705_ACCESS_H
#define _ES705_ACCESS_H

#define ES705_API_WORD(upper, lower) ((upper << 16) | lower)
#if defined(CONFIG_SND_SOC_ES705)
static struct es705_api_access es705_api_access[ES705_API_ADDR_MAX] = {
#elif defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES704_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)
static struct escore_api_access es705_api_access[ES705_API_ADDR_MAX] = {
#endif
	[ES705_MIC_CONFIG] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0002) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0002),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_AEC_MODE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0003) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0003),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_VEQ_ENABLE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0009) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0009),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_DEREVERB_ENABLE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0053) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0053),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_DEREVERB_GAIN] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0054) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0054),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_BWE_ENABLE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x004f) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x004f),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_BWE_HIGH_BAND_GAIN] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0050) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0050),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_BWE_POST_EQ_ENABLE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0052) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0052),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_SLIMBUS_LINK_MULTI_CHANNEL] = {
		.read_msg = { ES705_API_WORD(ES705_GET_DEV_PARAM, 0x0900) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_DEV_PARAM_ID, 0x0900),
			       ES705_API_WORD(ES705_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_POWER_STATE] = {
		.read_msg = { ES705_API_WORD(0x800f, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x9010, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
#ifndef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
		.val_max = 6,
#else
		.val_max = 8,
#endif
	},
	[ES705_FE_STREAMING] = {
		.read_msg = { ES705_API_WORD(0x8028, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x9028, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_PRESET] = {
		.read_msg = { ES705_API_WORD(0x8031, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x9031, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_ALGO_PROCESSING] = {
		.read_msg = { ES705_API_WORD(0x8043, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x801c, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_CHANGE_STATUS] = {
		.read_msg = { ES705_API_WORD(0x804f, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x804f, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 4,
	},
	[ES705_FW_FIRST_CHAR] = {
		.read_msg = { ES705_API_WORD(0x8020, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8020, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 255,
	},
	[ES705_FW_NEXT_CHAR] = {
		.read_msg = { ES705_API_WORD(0x8021, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8021, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 255,
	},
	[ES705_EVENT_RESPONSE] = {
		.read_msg = { ES705_API_WORD(0x801a, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x901a, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 4,
	},
	[ES705_VOICE_SENSE_ENABLE] = {
		.read_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_VOICE_SENSE_SET_KEYWORD] = {
		.read_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 4,
	},
	[ES705_VOICE_SENSE_EVENT] = {
		.read_msg = { ES705_API_WORD(0x806d, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 2,
	},
	[ES705_VOICE_SENSE_TRAINING_MODE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x5003) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x5003),
				   ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 2,
	},
	[ES705_VOICE_SENSE_DETECTION_SENSITIVITY] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x5004) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x5004),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 10,
	},
	[ES705_VOICE_ACTIVITY_DETECTION_SENSITIVITY] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x5005) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x5005),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 10,
	},
	[ES705_VOICE_SENSE_TRAINING_RECORD] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x5006) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x5006),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 2,
	},
	[ES705_VOICE_SENSE_TRAINING_STATUS] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x5007) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x5007),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 7,
	},
	[ES705_VOICE_SENSE_TRAINING_MODEL_LENGTH] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x500A) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x500A),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 75,
	},
	[ES705_VOICE_SENSE_DEMO_ENABLE] = {
		.read_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_VS_STORED_KEYWORD] = {
		.read_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_VS_INT_OSC_MEASURE_START] = {
		.read_msg = { ES705_API_WORD(0x8070, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x9070, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_VS_INT_OSC_MEASURE_STATUS] = {
		.read_msg = { ES705_API_WORD(0x8071, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8071, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES705_CVS_PRESET] = {
		.read_msg = { ES705_API_WORD(0x806F, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x906F, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES705_RX_ENABLE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x0075) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x0075),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES80X_AF_DUO_MODE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x2002) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x2002),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES80X_ENABLE_SDE_MODE] = {
		.read_msg = { ES705_API_WORD(ES705_GET_ALGO_PARAM, 0x202A) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(ES705_SET_ALGO_PARAM_ID, 0x202A),
			       ES705_API_WORD(ES705_SET_ALGO_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES80X_SDE_BEARING_VALUE] = {
		.read_msg = { ES705_API_WORD(ES80X_SDE_PARAMETER, 0x2000) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES80X_SDE_SALIENCE_VALUE] = {
		.read_msg = { ES705_API_WORD(ES80X_SDE_PARAMETER, 0x2001) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 32767,
	},
	[ES80X_SDE_RMS_ESTIMATE_VALUE] = {
		.read_msg = { ES705_API_WORD(ES80X_SDE_PARAMETER, 0x2002) },
		.read_msg_len = 4,
		.write_msg = { ES705_API_WORD(0x8000, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 32767,
	},
};

#endif /* _ES705_ACCESS_H */
