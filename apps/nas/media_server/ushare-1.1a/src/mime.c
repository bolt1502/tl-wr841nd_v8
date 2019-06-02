/*
 * mime.c : GeeXboX uShare media file MIME-type association.
 * Originally developped for the GeeXboX project.
 * Ref : http://freedesktop.org/wiki/Standards_2fshared_2dmime_2dinfo_2dspec
 * Copyright (C) 2005-2007 Benjamin Zores <ben@geexbox.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <stdlib.h>
#include <string.h>

#include "mime.h"
#include "ushare.h"

const struct mime_type_t MIME_Type_List[] = {
	
/* Video files */
{MIME_EXTENSION_ASF,  		UPNP_VIDEO, 	VIDEO_X_MS_ASF},
{MIME_EXTENSION_AVC,  		UPNP_VIDEO, 	VIDEO_AVI},
{MIME_EXTENSION_AVI,  		UPNP_VIDEO, 	VIDEO_AVI},
{MIME_EXTENSION_DV,   		UPNP_VIDEO, 	VIDEO_X_DV},
{MIME_EXTENSION_DIVX, 		UPNP_VIDEO, 	VIDEO_AVI},
{MIME_EXTENSION_WMV,  		UPNP_VIDEO, 	VIDEO_X_MS_WMV},
{MIME_EXTENSION_MJPG, 		UPNP_VIDEO, 	VIDEO_X_MOTION_JPEG},
{MIME_EXTENSION_MJPEG,		UPNP_VIDEO, 	VIDEO_X_MOTION_JPEG},
{MIME_EXTENSION_MPEG, 		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_MPG,  		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_MPE,  		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_MP2P, 		UPNP_VIDEO, 	VIDEO_MP2P},
{MIME_EXTENSION_VOB,  		UPNP_VIDEO, 	VIDEO_MP2P},
{MIME_EXTENSION_MP2T, 		UPNP_VIDEO, 	VIDEO_MP2T},
{MIME_EXTENSION_M1V,  		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_M2V,  		UPNP_VIDEO, 	VIDEO_MPEG2},
{MIME_EXTENSION_MPG2, 		UPNP_VIDEO, 	VIDEO_MPEG2},
{MIME_EXTENSION_MPEG2,		UPNP_VIDEO, 	VIDEO_MPEG2},
{MIME_EXTENSION_M4V,  		UPNP_VIDEO, 	VIDEO_MP4},
{MIME_EXTENSION_M4P,  		UPNP_VIDEO, 	VIDEO_MP4},
{MIME_EXTENSION_MP4PS,		UPNP_VIDEO, 	VIDEO_X_NERODIGITAL_PS},
{MIME_EXTENSION_TS,   		UPNP_VIDEO, 	VIDEO_MPEG2},
{MIME_EXTENSION_OGM,  		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_MKV,  		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_RMVB, 		UPNP_VIDEO, 	VIDEO_MPEG},
{MIME_EXTENSION_MOV,  		UPNP_VIDEO, 	VIDEO_QUICKTIME},
{MIME_EXTENSION_HDMOV,		UPNP_VIDEO, 	VIDEO_QUICKTIME},
{MIME_EXTENSION_QT,   		UPNP_VIDEO, 	VIDEO_QUICKTIME},
{MIME_EXTENSION_BIN,  		UPNP_VIDEO, 	VIDEO_MPEG2},
{MIME_EXTENSION_ISO,  		UPNP_VIDEO, 	VIDEO_MPEG2},

/* Audio files */
{MIME_EXTENSION_3GP, 		UPNP_AUDIO, 	VIDEO_QUICKTIME},
{MIME_EXTENSION_AAC, 		UPNP_AUDIO, 	AUDIO_X_AAC},
{MIME_EXTENSION_AC3, 		UPNP_AUDIO, 	AUDIO_X_AC3},
{MIME_EXTENSION_AIF, 		UPNP_AUDIO, 	AUDIO_AIFF},
{MIME_EXTENSION_AIFF,		UPNP_AUDIO, 	AUDIO_AIFF},
{MIME_EXTENSION_AT3P,		UPNP_AUDIO, 	AUDIO_X_ATRAC3},
{MIME_EXTENSION_AU,  		UPNP_AUDIO, 	AUDIO_BASIC},
{MIME_EXTENSION_SND, 		UPNP_AUDIO, 	AUDIO_BASIC},
{MIME_EXTENSION_DTS, 		UPNP_AUDIO, 	AUDIO_X_DTS},
{MIME_EXTENSION_RMI, 		UPNP_AUDIO, 	AUDIO_MIDI},
{MIME_EXTENSION_MID, 		UPNP_AUDIO, 	AUDIO_MIDI},
{MIME_EXTENSION_MP1, 		UPNP_AUDIO, 	AUDIO_MP1},
{MIME_EXTENSION_MP2, 		UPNP_AUDIO, 	AUDIO_MP2},
{MIME_EXTENSION_MP3, 		UPNP_AUDIO, 	AUDIO_MP2},
{MIME_EXTENSION_MP4, 		UPNP_AUDIO, 	AUDIO_MP4},
{MIME_EXTENSION_M4A, 		UPNP_AUDIO, 	AUDIO_MP4},
{MIME_EXTENSION_OGG, 		UPNP_AUDIO, 	AUDIO_X_OGG},
{MIME_EXTENSION_WAV, 		UPNP_AUDIO, 	AUDIO_WAV},
{MIME_EXTENSION_PCM, 		UPNP_AUDIO, 	AUDIO_L16},
{MIME_EXTENSION_LPCM,		UPNP_AUDIO, 	AUDIO_L16},
{MIME_EXTENSION_L16, 		UPNP_AUDIO, 	AUDIO_L16},
{MIME_EXTENSION_WMA, 		UPNP_AUDIO, 	AUDIO_X_MS_WMA},
{MIME_EXTENSION_MKA, 		UPNP_AUDIO, 	AUDIO_MP2},
{MIME_EXTENSION_RA,  		UPNP_AUDIO, 	AUDIO_X_PN_REALAUDIO},
{MIME_EXTENSION_RM,  		UPNP_AUDIO, 	AUDIO_X_PN_REALAUDIO},
{MIME_EXTENSION_RAM, 		UPNP_AUDIO, 	AUDIO_X_PN_REALAUDIO},
{MIME_EXTENSION_FLAC,		UPNP_AUDIO, 	AUDIO_X_FLAC},

/* Images files */
{MIME_EXTENSION_BMP,  		UPNP_PHOTO, 	IMAGE_BMP},
{MIME_EXTENSION_ICO,  		UPNP_PHOTO, 	IMAGE_X_ICON},
{MIME_EXTENSION_GIF,  		UPNP_PHOTO, 	IMAGE_GIF},
{MIME_EXTENSION_JPEG, 		UPNP_PHOTO, 	IMAGE_JPEG},
{MIME_EXTENSION_JPG,  		UPNP_PHOTO, 	IMAGE_JPEG},
{MIME_EXTENSION_JPE,  		UPNP_PHOTO, 	IMAGE_JPEG},
{MIME_EXTENSION_PCD,  		UPNP_PHOTO, 	IMAGE_X_MS_BMP},
{MIME_EXTENSION_PNG,  		UPNP_PHOTO, 	IMAGE_PNG},
{MIME_EXTENSION_PNM,  		UPNP_PHOTO, 	IMAGE_X_PORTABLE_ANYMAP},
{MIME_EXTENSION_PPM,  		UPNP_PHOTO, 	IMAGE_X_PORTABLE_PIXMAP},
{MIME_EXTENSION_QTI,  		UPNP_PHOTO, 	IMAGE_X_QUICKTIME},
{MIME_EXTENSION_QTF,  		UPNP_PHOTO, 	IMAGE_X_QUICKTIME},
{MIME_EXTENSION_QTIF, 		UPNP_PHOTO, 	IMAGE_X_QUICKTIME},
{MIME_EXTENSION_TIF,  		UPNP_PHOTO, 	IMAGE_TIFF},
{MIME_EXTENSION_TIFF, 		UPNP_PHOTO, 	IMAGE_TIFF},

/* Playlist files */
{MIME_EXTENSION_PLS, 		UPNP_PLAYLIST, 	AUDIO_X_SCPLS},
{MIME_EXTENSION_M3U, 		UPNP_PLAYLIST, 	AUDIO_MPEGURL},
{MIME_EXTENSION_ASX, 		UPNP_PLAYLIST, 	VIDEO_X_MS_ASF},

/* Subtitle Text files */
{MIME_EXTENSION_SRT, 		UPNP_TEXT, 		TEXT_SRT}, /* SubRip */
{MIME_EXTENSION_SSA, 		UPNP_TEXT, 		TEXT_SSA}, /* SubStation Alpha */
{MIME_EXTENSION_STL, 		UPNP_TEXT, 		TEXT_SRT}, /* Spruce */
{MIME_EXTENSION_PSB, 		UPNP_TEXT, 		TEXT_PSB}, /* PowerDivX */
{MIME_EXTENSION_PJS, 		UPNP_TEXT, 		TEXT_PJS}, /* Phoenix Japanim */
{MIME_EXTENSION_SUB, 		UPNP_TEXT, 		TEXT_SUB}, /* MicroDVD */
{MIME_EXTENSION_IDX, 		UPNP_TEXT, 		TEXT_IDX}, /* VOBsub */
{MIME_EXTENSION_DKS, 		UPNP_TEXT, 		TEXT_DKS}, /* DKS */
{MIME_EXTENSION_SCR, 		UPNP_TEXT, 		TEXT_SCR}, /* MACsub */
{MIME_EXTENSION_TTS, 		UPNP_TEXT, 		TEXT_TTS}, /* TurboTitler */
{MIME_EXTENSION_VSF, 		UPNP_TEXT, 		TEXT_VSF}, /* ViPlay */
{MIME_EXTENSION_ZEG, 		UPNP_TEXT, 		TEXT_ZEG}, /* ZeroG */
{MIME_EXTENSION_MPL, 		UPNP_TEXT, 		TEXT_MPL}, /* MPL */

/* Miscellaneous text files */
{MIME_EXTENSION_BUP, 		UPNP_TEXT, 		TEXT_BUP}, /* DVD backup */
{MIME_EXTENSION_IFO, 		UPNP_TEXT, 		TEXT_IFO}, /* DVD information */
{INVALID_MIME_EXTENSION,	INVALID_MIME_CLASS, INVALID_MIME_PROTOCOL},

};

/* mime information list. we will use the index to save the mime info. by HouXB, 06Feb12 */
char* mime_extention_list[] =
{
	/* video files. */
	"asf",  
	"avc",  
	"avi",  
	"dv",   
	"divx", 
	"wmv",  
	"mjpg", 
	"mjpeg",
	"mpeg", 
	"mpg",  
	"mpe",  
	"mp2p", 
	"vob",  
	"mp2t", 
	"m1v",  
	"m2v",  
	"mpg2", 
	"mpeg2",
	"m4v",  
	"m4p",  
	"mp4ps",
	"ts",   
	"ogm",  
	"mkv",  
	"rmvb", 
	"mov",  
	"hdmov",
	"qt",   
	"bin",  
	"iso",  

	/* audio files. */
	"3gp", 
	"aac", 
	"ac3", 
	"aif", 
	"aiff",
	"at3p",
	"au",  
	"snd", 
	"dts", 
	"rmi", 
	"mid", 
	"mp1", 
	"mp2", 
	"mp3", 
	"mp4", 
	"m4a", 
	"ogg", 
	"wav", 
	"pcm", 
	"lpcm",
	"l16", 
	"wma", 
	"mka", 
	"ra",  
	"rm",  
	"ram", 
	"flac",

	/* image files. */
	"bmp", 
	"ico", 
	"gif", 
	"jpeg",
	"jpg", 
	"jpe", 
	"pcd", 
	"png", 
	"pnm", 
	"ppm", 
	"qti", 
	"qtf", 
	"qtif",
	"tif", 
	"tiff",

	/* playlist files. */
	"pls",
	"m3u",
	"asx",

	/* subtitle text files. */
	"srt",
	"ssa",
	"stl",
	"psb",
	"pjs",
	"sub",
	"idx",
	"dks",
	"scr",
	"tts",
	"vsf",
	"zeg",
	"mpl",

	/* Miscellaneous text files. */
	"bup",
	"ifo",
};

char* mime_class_list[] = 
{
	"object.item.videoItem",
	"object.item.audioItem.musicTrack",
	"object.item.imageItem.photo",
	"object.item.playlistItem",
	"object.item.textItem",
	"object.container.storageFolder",
};

char* mime_protocol_list[] = 
{
	/* video */
	"x-ms-asf",
	"avi",
	"x-dv",
	"x-ms-wmv",
	"x-motion-jpeg",	
	"mpeg",
	"mp2p",
	"mp2t",
	"mpeg2",
	"mp4",
	"x-nerodigital-ps",
	"quicktime",

	/* audio */
	"3gpp",
	"x-aac",
	"x-ac3",
	"aiff",
	"x-atrac3",
	"basic",
	"x-dts",
	"midi",
	"mp1",
	"mp2",
	"mpeg",
	"mp4",
	"x-ogg",
	"wav",
	"l16",
	"x-ms-wma",
	"x-pn-realaudio",
	"x-flac",
	"x-scpls",
	"mpegurl",
	/* image */
	"bmp",
	"x-icon",
	"gif",
	"jpeg",
	"x-ms-bmp",
	"png",
	"x-portable-anymap",
	"x-portable-pixmap",
	"x-quicktime",
	"tiff",
	
	/* text */	
	"srt", /* SubRip */
	"ssa", /* SubStation Alpha */
	"psb", /* PowerDivX */
	"pjs", /* Phoenix Japanim */
	"sub", /* MicroDVD */
	"idx", /* VOBsub */
	"dks", /* DKS */
	"scr", /* MACsub */
	"tts", /* TurboTitler */
	"vsf", /* ViPlay */
	"zeg", /* ZeroG */
	"mpl", /* MPL */
	"bup", /* DVD backup */
	"ifo", /* DVD information */
};

#define MIME_STR_LEN_MAX 512

char *mime_get_extension(struct mime_type_t *mime)
{
	if (!mime)
		return NULL;
	
	if ((mime->extension == INVALID_MIME_EXTENSION)
		|| (mime->extension >= MIME_EXTENSION_LIST_MAX))
		return NULL;
	
	return mime_extention_list[mime->extension];
}

char *mime_get_protocol (struct mime_type_t *mime)
{
	char protocol[MIME_STR_LEN_MAX];

	if (!mime)
		return NULL;

	if (mime->mime_protocol > MIME_PROTOCOL_LIST_MAX)
		return NULL;
	
	switch(mime->mime_class)
	{
		case UPNP_VIDEO:
			snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:video/%s:*",
						mime_protocol_list[mime->mime_protocol]);
			break;
		case UPNP_AUDIO:
			snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:audio/%s:*",
						mime_protocol_list[mime->mime_protocol]);			
			break;
		case UPNP_PHOTO:
			snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:image/%s:*",
						mime_protocol_list[mime->mime_protocol]);			
			
			break;
		case UPNP_PLAYLIST:
			if(mime->extension == MIME_EXTENSION_ASX)
			{
				snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:video/%s:*",
						mime_protocol_list[mime->mime_protocol]);			

			}
			else
			{
				snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:audio/%s:*",
						mime_protocol_list[mime->mime_protocol]);			

			}
			
			break;
		case UPNP_TEXT:
			snprintf (protocol, MIME_STR_LEN_MAX, 
						"http-get:*:text/%s:*",
						mime_protocol_list[mime->mime_protocol]);			

			break;
		default:
			return NULL;
	}

	return strdup (protocol);
}

char *mime_get_class(struct mime_type_t *mime)
{
	char mime_class[MIME_STR_LEN_MAX];

	if (!mime)
		return NULL;

	if (mime->mime_class > MIME_CLASS_LIST_MAX)
		return NULL;

	snprintf (mime_class, MIME_STR_LEN_MAX, 
				"%s", mime_protocol_list[mime->mime_protocol]);
	return strdup (mime_class);
}

