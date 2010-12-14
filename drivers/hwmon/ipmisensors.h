/*
 *  ipmisensors.h -	lm_sensors interface to IPMI sensors.
 *
 *  Copyright (C) 2004-2007 Yani Ioannou <yani.ioannou@gmail.com>
 *
 *  Adapted from bmcsensors (lm-sensors for linux 2.4)
 *  bmcsensors (C) Mark D. Studebaker <mdsxyz123@yahoo.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/ipmi.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* SDR defs */
#define STYPE_TEMP			0x01
#define STYPE_VOLT			0x02
#define STYPE_CURR			0x03
#define STYPE_FAN			0x04

#define SDR_LIMITS 			8
#define SDR_MAX_ID_LENGTH 		16
#define SDR_MAX_UNPACKED_ID_LENGTH 	((SDR_MAX_ID_LENGTH * 4 / 3) + 2)

/* the last sensor type we are interested in */
#define STYPE_MAX			4

#define IPMI_SDR_SIZE			67
#define IPMI_CHUNK_SIZE 		16

#define MAX_FILENAME_LENGTH		30

struct ipmisensors_device_attribute {
	struct device_attribute dev_attr;
	struct sdrdata *sdr;
};
#define to_ipmisensors_dev_attr(_dev_attr) \
	container_of(_dev_attr, struct ipmisensors_device_attribute, dev_attr)

#define IPMISENSORS_DEVICE_ATTR(_name,_mode,_show,_store,_index)	\
struct ipmisensors_attribute sensor_dev_attr_##_name = {	\
	.dev_attr =	__ATTR(_name,_mode,_show,_store),	\
	.index =	_index,					\
}

struct ipmisensors_bmc_device_attribute {
	struct device_attribute dev_attr;
	struct ipmisensors_bmc_data *bmc;
};
#define to_ipmisensors_bmc_dev_attr(_dev_attr) \
	container_of(_dev_attr, struct ipmisensors_bmc_device_attribute, dev_attr)

/**
 * &struct_sdrdata stores the IPMI Sensor Data Record (SDR) data, as recieved from the BMC, along with the corresponding sysfs attributes
 */
struct sdrdata {
	struct list_head list;
	/* retrieved from SDR, not expected to change */
	/* Sensor Type Code */
	u8 stype;
	u8 number;
	/* Sensor Capability Code */
	u8 capab;
	u16 thresh_mask;
	u8 format;
	u8 linear;
	s16 m;
	s16 b;
	u8 k;
	u8 nominal;
	u8 limits[SDR_LIMITS];
	/* index into limits for reported upper and lower limit */
	int lim1, lim2;
	u8 lim1_write, lim2_write;
	u8 string_type;
	u8 id_length;
	u8 id[SDR_MAX_ID_LENGTH];
	/* retrieved from reading */
	u8 reading;
	u8 status;
	u8 thresholds;
	/* sensor's bmc */
	struct ipmisensors_bmc_data *bmc;
	/* sysfs entries */
	struct ipmisensors_device_attribute attr;
	char *attr_name;
	struct ipmisensors_device_attribute attr_min;
	char *attr_min_name;
	struct ipmisensors_device_attribute attr_max;
	char *attr_max_name;
	struct ipmisensors_device_attribute attr_label;
	char *attr_label_name;

};

/**
 * &struct_ipmisensors_data stores the data for the ipmisensors driver.
 */
struct ipmisensors_data {
	/* Driver struct */
	char *driver_name;

	/* Linked list of ipmisensors_bmc_data structs, one for each BMC */
	struct list_head bmc_data;

	/* Number of ipmi interfaces (and hence ipmisensors_data structs). */
	int interfaces;

	/* IPMI kernel interface - SMI watcher */
	struct ipmi_smi_watcher smi_watcher;

	/* IPMI kernel interface - user handlers */
	struct ipmi_user_hndl ipmi_hndlrs;

	/* Cache manager for sdrdata cache */
	struct kmem_cache *sdrdata_cache;

	/* Cache manager for ipmi_sensor_device_attribute cache */
	struct kmem_cache *sysfsattr_cache;
};

/**
 * &states: enumeration of state codes for a bmc specific ipmisensors
 */
enum states {
	STATE_INIT,
	STATE_RESERVE,
	STATE_SDR,
	STATE_SDRPARTIAL,
	STATE_READING,
	STATE_UNCANCEL,
	STATE_SYSTABLE,
	STATE_DONE
};

/**
 * &struct_ipmisensors_bmc_data stores the data for a particular IPMI BMC.
 */
struct ipmisensors_bmc_data {
	struct list_head list;

	/* The IPMI interface number */
	int interface_id;

	/* The IPMI address */
	struct ipmi_addr address;

	/* List of sdrdata structs (sdrs) recieved from the BMC */
	struct list_head sdrs;

	/* Count of the number of sdrs stored in the sdr list */
	int sdr_count;

	/* next message id */
	int msgid;

	/* The ipmi interface 'user' used to access this particular bmc */
	ipmi_user_t user;

	/* BMC IPMI Version (major) */
	unsigned char ipmi_version_major;

	/* BMC IPMI Version (minor) */
	unsigned char ipmi_version_minor;

	/* The size of the SDR request message */
	int ipmi_sdr_partial_size;

	/* transmit message buffer */
	struct kernel_ipmi_msg tx_message;

	/* ipmi transmited data buffer */
	unsigned char tx_msg_data[IPMI_MAX_MSG_LENGTH + 50];	/* why the +50 in bmcsensors? */

	/* ipmi recieved data buffer */
	unsigned char rx_msg_data[IPMI_MAX_MSG_LENGTH + 50];

	/* current recieve buffer offset */
	int rx_msg_data_offset;

	/* The id of then next SDR record to read during update cycle */
	u16 nextrecord;

	/* BMC SDR Reservation ID */
	u16 resid;

	/* Alarm status */
	u8 alarms;

	/* The cumalative error count for this bmc */
	int errorcount;

	/* The current state of this bmc w.r.t. ipmisensors (see enum states) */
	int state;

	/* The current sdr for which a reading is pending */
	struct sdrdata *current_sdr;

	/* The BMC's device struct */
	struct device *dev;

	/* hwmon device */
	struct device *hwmon_dev;

	/* hwmon device name */
	struct device_attribute name_attr;

	/* alarms attribute */
	struct ipmisensors_bmc_device_attribute alarms_attr;

	/* update_period attribute */
	struct ipmisensors_bmc_device_attribute update_attr;

	/* lower bound on time between updates (in seconds) */
	unsigned int update_period;

	/* semaphore used to do a headcount of the SDR readings we are waiting
	 * on in a given bmc update */
	struct semaphore update_semaphore;

	/* bmc's work struct for updating sensors */
	struct delayed_work update_work;

	/* bmc's work struct for building the sysfs workqueue */
	struct work_struct sysfs_work;
};
