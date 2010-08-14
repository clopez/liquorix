/*
 *   lirc_imon.h:  LIRC/VFD/LCD driver for SoundGraph iMON IR/VFD/LCD
 *		   including the iMON PAD model
 *
 *   Copyright(C) 2004  Venky Raju(dev@venky.ws)
 *   Copyright(C) 2009  Jarod Wilson <jarod@wilsonet.com>
 *
 *   lirc_imon is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

static const struct {
	u32 hw_code;
	u16 keycode;
} imon_remote_key_table[] = {
	/* keys sorted mostly by frequency of use to optimize lookups */
	{ 0x2a8195b7, KEY_REWIND },
	{ 0x298315b7, KEY_REWIND },
	{ 0x2b8115b7, KEY_FASTFORWARD },
	{ 0x2b8315b7, KEY_FASTFORWARD },
	{ 0x2b9115b7, KEY_PREVIOUS },
	{ 0x298195b7, KEY_NEXT },

	{ 0x2a8115b7, KEY_PLAY },
	{ 0x2a8315b7, KEY_PLAY },
	{ 0x2a9115b7, KEY_PAUSE },
	{ 0x2b9715b7, KEY_STOP },
	{ 0x298115b7, KEY_RECORD },

	{ 0x01008000, KEY_UP },
	{ 0x01007f00, KEY_DOWN },
	{ 0x01000080, KEY_LEFT },
	{ 0x0100007f, KEY_RIGHT },

	{ 0x2aa515b7, KEY_UP },
	{ 0x289515b7, KEY_DOWN },
	{ 0x29a515b7, KEY_LEFT },
	{ 0x2ba515b7, KEY_RIGHT },

	{ 0x0200002c, KEY_SPACE }, /* Select/Space */
	{ 0x02000028, KEY_ENTER },
	{ 0x288195b7, KEY_EXIT },
	{ 0x02000029, KEY_ESC },
	{ 0x0200002a, KEY_BACKSPACE },

	{ 0x2b9595b7, KEY_MUTE },
	{ 0x28a395b7, KEY_VOLUMEUP },
	{ 0x28a595b7, KEY_VOLUMEDOWN },
	{ 0x289395b7, KEY_CHANNELUP },
	{ 0x288795b7, KEY_CHANNELDOWN },

	{ 0x0200001e, KEY_NUMERIC_1 },
	{ 0x0200001f, KEY_NUMERIC_2 },
	{ 0x02000020, KEY_NUMERIC_3 },
	{ 0x02000021, KEY_NUMERIC_4 },
	{ 0x02000022, KEY_NUMERIC_5 },
	{ 0x02000023, KEY_NUMERIC_6 },
	{ 0x02000024, KEY_NUMERIC_7 },
	{ 0x02000025, KEY_NUMERIC_8 },
	{ 0x02000026, KEY_NUMERIC_9 },
	{ 0x02000027, KEY_NUMERIC_0 },

	{ 0x02200025, KEY_NUMERIC_STAR },
	{ 0x02200020, KEY_NUMERIC_POUND },

	{ 0x2b8515b7, KEY_VIDEO },
	{ 0x299195b7, KEY_AUDIO },
	{ 0x2ba115b7, KEY_CAMERA },
	{ 0x28a515b7, KEY_TV },
	{ 0x29a395b7, KEY_DVD },
	{ 0x29a295b7, KEY_DVD },

	/* the Menu key between DVD and Subtitle on the RM-200... */
	{ 0x2ba385b7, KEY_MENU },
	{ 0x2ba395b7, KEY_MENU },

	{ 0x288515b7, KEY_BOOKMARKS },
	{ 0x2ab715b7, KEY_MEDIA }, /* Thumbnail */
	{ 0x298595b7, KEY_SUBTITLE },
	{ 0x2b8595b7, KEY_LANGUAGE },

	{ 0x29a595b7, KEY_ZOOM },
	{ 0x2aa395b7, KEY_SCREEN }, /* FullScreen */

	{ 0x299115b7, KEY_KEYBOARD },
	{ 0x299135b7, KEY_KEYBOARD },

	{ 0x01010000, BTN_LEFT },
	{ 0x01020000, BTN_RIGHT },
	{ 0x01010080, BTN_LEFT },
	{ 0x01020080, BTN_RIGHT },

	{ 0x2a9395b7, KEY_CYCLEWINDOWS }, /* TaskSwitcher */
	{ 0x2b8395b7, KEY_TIME }, /* Timer */

	{ 0x289115b7, KEY_POWER },
	{ 0x29b195b7, KEY_EJECTCD }, /* the one next to play */
	{ 0x299395b7, KEY_EJECTCLOSECD }, /* eject (above TaskSwitcher) */

	{ 0x02800000, KEY_MENU }, /* Left Menu */
	{ 0x02000065, KEY_COMPOSE }, /* RightMenu */
	{ 0x2ab195b7, KEY_PROG1 }, /* Go */
	{ 0x29b715b7, KEY_DASHBOARD }, /* AppLauncher */
};

static const struct {
	u32 hw_code;
	u16 keycode;
} imon_mce_key_table[] = {
	/* keys sorted mostly by frequency of use to optimize lookups */
	{ 0x800f8415, KEY_REWIND },
	{ 0x800f8414, KEY_FASTFORWARD },
	{ 0x800f841b, KEY_PREVIOUS },
	{ 0x800f841a, KEY_NEXT },

	{ 0x800f8416, KEY_PLAY },
	{ 0x800f8418, KEY_PAUSE },
	{ 0x800f8418, KEY_PAUSE },
	{ 0x800f8419, KEY_STOP },
	{ 0x800f8417, KEY_RECORD },

	{ 0x02000052, KEY_UP },
	{ 0x02000051, KEY_DOWN },
	{ 0x02000050, KEY_LEFT },
	{ 0x0200004f, KEY_RIGHT },

	{ 0x02000028, KEY_ENTER },
/* the OK and Enter buttons decode to the same value
	{ 0x02000028, KEY_OK }, */
	{ 0x0200002a, KEY_EXIT },
	{ 0x02000029, KEY_DELETE },

	{ 0x800f840e, KEY_MUTE },
	{ 0x800f8410, KEY_VOLUMEUP },
	{ 0x800f8411, KEY_VOLUMEDOWN },
	{ 0x800f8412, KEY_CHANNELUP },
	{ 0x800f8413, KEY_CHANNELDOWN },

	{ 0x0200001e, KEY_NUMERIC_1 },
	{ 0x0200001f, KEY_NUMERIC_2 },
	{ 0x02000020, KEY_NUMERIC_3 },
	{ 0x02000021, KEY_NUMERIC_4 },
	{ 0x02000022, KEY_NUMERIC_5 },
	{ 0x02000023, KEY_NUMERIC_6 },
	{ 0x02000024, KEY_NUMERIC_7 },
	{ 0x02000025, KEY_NUMERIC_8 },
	{ 0x02000026, KEY_NUMERIC_9 },
	{ 0x02000027, KEY_NUMERIC_0 },

	{ 0x02200025, KEY_NUMERIC_STAR },
	{ 0x02200020, KEY_NUMERIC_POUND },

	{ 0x800f8446, KEY_TV },
	{ 0x800f8447, KEY_AUDIO },
	{ 0x800f8448, KEY_PVR }, /* RecordedTV */
	{ 0x800f8449, KEY_CAMERA },
	{ 0x800f844a, KEY_VIDEO },
	{ 0x800f8424, KEY_DVD },
	{ 0x800f8425, KEY_TUNER }, /* LiveTV */

	{ 0x800f845b, KEY_RED },
	{ 0x800f845c, KEY_GREEN },
	{ 0x800f845d, KEY_YELLOW },
	{ 0x800f845e, KEY_BLUE },

	{ 0x800f840f, KEY_INFO },
	{ 0x800f8426, KEY_EPG }, /* Guide */
	{ 0x800f845a, KEY_SUBTITLE }, /* Caption */

	{ 0x800f840c, KEY_POWER },
	{ 0x800f840d, KEY_PROG1 }, /* Windows MCE button */

};

static const struct {
	u64 hw_code;
	u16 keycode;
} imon_panel_key_table[] = {
	{ 0x000000000f000fee, KEY_PROG1 }, /* Go */
	{ 0x000000001f000fee, KEY_AUDIO },
	{ 0x0000000020000fee, KEY_VIDEO },
	{ 0x0000000021000fee, KEY_CAMERA },
	{ 0x0000000027000fee, KEY_DVD },
/* the TV key on my panel is broken, doesn't work under any OS
	{ 0x0000000000000fee, KEY_TV }, */
	{ 0x0000000005000fee, KEY_PREVIOUS },
	{ 0x0000000007000fee, KEY_REWIND },
	{ 0x0000000004000fee, KEY_STOP },
	{ 0x000000003c000fee, KEY_PLAYPAUSE },
	{ 0x0000000008000fee, KEY_FASTFORWARD },
	{ 0x0000000006000fee, KEY_NEXT },
	{ 0x0000000100000fee, KEY_RIGHT },
	{ 0x0000010000000fee, KEY_LEFT },
	{ 0x000000003d000fee, KEY_SELECT },
	{ 0x0001000000000fee, KEY_VOLUMEUP },
	{ 0x0100000000000fee, KEY_VOLUMEDOWN },
	{ 0x0000000001000fee, KEY_MUTE },
};
