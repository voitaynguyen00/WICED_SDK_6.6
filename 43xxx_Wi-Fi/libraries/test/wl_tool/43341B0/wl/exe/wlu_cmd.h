/*
 * Command structure for wl command line utility
 *
 * Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * $Id: wlu_cmd.h 241182 2011-02-17 21:50:03Z gmo $
 */

#ifndef _wlu_cmd_h_
#define _wlu_cmd_h_

typedef struct cmd cmd_t;
typedef int (cmd_func_t)(void *wl, cmd_t *cmd, char **argv);

/* generic command line argument handler */
struct cmd {
	const char *name;
	cmd_func_t *func;
	int get;
	int set;
	const char *help;
};

/* list of command line arguments */
extern cmd_t wl_cmds[];
extern cmd_t wl_varcmd;

/* per-port ioctl handlers */
extern int wl_get(void *wl, int cmd, void *buf, int len);
extern int wl_set(void *wl, int cmd, void *buf, int len);

/*
 * Flags for command categories.  A command may belong to
 * multiple categories.  These are used in bitmaps, so be careful
 * to keep the macro value (2 ^ n) and the array indexes (n)
 * consistent.
 */

#define CMD_PHY               0x1
#define CMD_CHAN              0x2
#define CMD_RATE              0x4
#define CMD_POWER             0x8
#define CMD_MAC               0x10
#define CMD_MGMT              0x20
#define CMD_SEC               0x40
#define CMD_WME               0x80
#define CMD_MON               0x100
#define CMD_AP                0x200
#define CMD_STA               0x400
#define CMD_BOARD             0x800
#define CMD_ADMIN             0x1000
#define CMD_DEV               0x2000
#define CMD_DEP               0x4000
#define CMD_UNCAT             0x8000

#define CMD_ALL               0xffff

/* Initializer for category string array */

#define CMD_CATEGORY_STRINGS_INIT { \
	"phy",   \
	"chan",  \
	"rate",  \
	"power", \
	"mac",   \
	"mgmt",  \
	"sec",   \
	"wme",   \
	"mon",   \
	"ap",    \
	"sta",   \
	"board", \
	"admin", \
	"dev",   \
	"dep",   \
	"uncat", \
	"" }

extern const char *wl_cmd_category_strings[];
extern const int wl_cmd_category_count;

/* Initializer for category description strings array */
#define CMD_CATEGORY_DESC_INIT { \
	"PHY and radio; speed, band, etc", \
	"Channel; subclass of phy",  \
	"Rate; subclass of phy, a/b/g",  \
	"Power; subclass of phy", \
	"MAC; Media access",   \
	"Management, association, IE, etc",  \
	"Security; subclass of mgmt",   \
	"WME; media extensions",   \
	"Monitoring device (counters, etc)",   \
	"AP subclass of mgmt", \
	"STA subclass of mgmt", \
	"Board, hardware", \
	"Administration; software, UI, diags", \
	"Device; low level control", \
	"Deprecated", \
	"Uncategorized so far", \
	"" }

extern const char *wl_cmd_category_desc[];

/*
 *
 * IO variable information
 *
 */

/* Supplemental IO variable info structure */
typedef const struct wlu_iov_info_s {
	const char *name;
	uint32 cat;  /* Category flags; same as command categories */
	uint32 flags; /* See below */
	int dflt;  /* Only for integers; see flags */
	const char *desc;  /* Description */
} wlu_iov_info_t;

/* Flags for wlu_iov_info_t */
#define WLU_IOVI_READ_ONLY         0x1   /* Known to be read only */
#define WLU_IOVI_WRITE_ONLY        0x2   /* Known to be write only */
#define WLU_IOVI_BCM_INTERNAL      0x4   /* Known to be BCM internal */
#define WLU_IOVI_DEFAULT_VALID     0x8   /* Default value in structure is valid */

extern wlu_iov_info_t wlu_iov_info[];
extern int wlu_iov_info_count;

#define WLU_IOV_BLOCK_LEN 10

#define WLU_MOD_NAME_MAX 16
#define WLU_MOD_NAME_BYTES 16

#endif /* _wlu_cmd_h_ */