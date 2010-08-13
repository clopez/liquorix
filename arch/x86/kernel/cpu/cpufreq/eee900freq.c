/*
 * Experimental cpu frequency scaling driver for the eeepc 900
 *
 * Copyright (C) 2008  Cristiano Prisciandaro <cristiano.p@solnet.ch>
 *
 * This driver is based on the (experimental) finding that the
 * eeepc bios exposes a method to underclock the bus/cpu.
 *
 * It seems to work fine with the following BIOS versions:
 * 0501, 0601, 0704 and 0802.
 *
 * Parts of this code are from
 *  asus_acpi.c Copyright (C) Julien Lerouge, Karol Kozimor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * BIG FAT DISCLAIMER: experimental code. Possibly *dangerous*
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq.h>
#include <acpi/acpi_drivers.h>
#include <acpi/acpi_bus.h>

#define MNAME                    "eee900freq:"
#define ASUS_HOTK_PREFIX         "\\_SB.ATKD"
#define ASUS_CPUFV_READ_METHOD   "CFVG"
#define ASUS_CPUFV_WRITE_METHOD  "CFVS"

static acpi_handle handle;
static unsigned int cpufreq_eee900_get(unsigned int cpu);

/* available frequencies */
static struct cpufreq_frequency_table eee900freq_table[] = {
	{0, 630000},
	{1, 900000},
	{0, CPUFREQ_TABLE_END}
};

struct eee900_acpi_value {
	int frequency;
	int value;
};

static struct eee900_acpi_value eee900_acpi_values_table[] = {
	{630000, 1},
	{900000, 0}
};

/* read from the acpi handle (from asus_acpi.c) */
static int read_eee900_acpi_int(acpi_handle handle, const char *method,
				int *val)
{
	struct acpi_buffer output;
	union acpi_object out_obj;
	acpi_status status;

	output.length = sizeof(out_obj);
	output.pointer = &out_obj;
	status = acpi_evaluate_object(handle, (char *)method, NULL, &output);
	*val = out_obj.integer.value;
	return status == AE_OK && out_obj.type == ACPI_TYPE_INTEGER;
}

/* write to the acpi handle (from asus_acpi.c) */
static int write_eee900_acpi_int(acpi_handle handle, const char *method,
				 int val, struct acpi_buffer *output)
{
	struct acpi_object_list params;
	union acpi_object in_obj;
	acpi_status status;

	params.count = 1;
	params.pointer = &in_obj;
	in_obj.type = ACPI_TYPE_INTEGER;
	in_obj.integer.value = val;
	status = acpi_evaluate_object(handle, (char *)method, &params, output);
	return status == AE_OK;
}

/* return the current frequency as in acpi */
static unsigned int cpufreq_eee900_get(unsigned int cpu)
{
	int value;

	if (!read_eee900_acpi_int(handle, ASUS_CPUFV_READ_METHOD, &value)) {
		printk(KERN_WARNING MNAME
		       "unable to read current frequency from "
		       ASUS_CPUFV_READ_METHOD "\n");
		return -EINVAL;
	}

	switch (value) {
	case 0x200:
		return 900000;
	case 0x201:
		return 630000;
	}

	return 0;
}

static void cpufreq_eee900_set_freq(unsigned int index)
{
	struct cpufreq_freqs freqs;

	freqs.old = cpufreq_eee900_get(0);
	freqs.new = eee900freq_table[index].frequency;
	freqs.cpu = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if (!write_eee900_acpi_int(handle, ASUS_CPUFV_WRITE_METHOD,
				   eee900_acpi_values_table[index].value, NULL))
		printk(KERN_WARNING "unable to set new frequency: val=%x",
		       eee900_acpi_values_table[index].value);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return;
}

static int cpufreq_eee900_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	unsigned int newstate = 0;

	if (cpufreq_frequency_table_target
	    (policy, &eee900freq_table[0], target_freq, relation, &newstate))
		return -EINVAL;

	cpufreq_eee900_set_freq(newstate);

	return 0;
}

static int cpufreq_eee900_cpu_init(struct cpufreq_policy *policy)
{

	unsigned int cfreq;

	cfreq = cpufreq_eee900_get(policy->cpu);

	cpufreq_frequency_table_get_attr(eee900freq_table, policy->cpu);

	/* cpuinfo and default policy values */
	policy->cpuinfo.transition_latency = 1000000;	/* assumed */
	policy->cur = cfreq;

	return cpufreq_frequency_table_cpuinfo(policy, &eee900freq_table[0]);
}

static int cpufreq_eee900_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static int cpufreq_eee900_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, &eee900freq_table[0]);
}

static struct freq_attr *eee900freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver eee900freq_driver = {
	.verify	= cpufreq_eee900_verify,
	.target	= cpufreq_eee900_target,
	.init	= cpufreq_eee900_cpu_init,
	.exit	= cpufreq_eee900_cpu_exit,
	.get	= cpufreq_eee900_get,
	.name	= "eee900freq",
	.owner	= THIS_MODULE,
	.attr	= eee900freq_attr,
};

static int __init cpufreq_eee900_init(void)
{
	struct cpuinfo_x86 *c = &cpu_data(0);
	acpi_status status;
	int ret;
	int test;

	if (c->x86_vendor != X86_VENDOR_INTEL)
		return -ENODEV;

	status = acpi_get_handle(NULL, ASUS_HOTK_PREFIX, &handle);

	if (ACPI_FAILURE(status)) {
		printk(KERN_INFO MNAME "unable to get acpi handle.\n");
		handle = NULL;
		return -ENODEV;
	}

	/* check if the control method is supported */
	if (!read_eee900_acpi_int(handle, ASUS_CPUFV_READ_METHOD, &test)) {
		printk(KERN_INFO "Get control method test failed\n");
		return -ENODEV;
	}

	ret = cpufreq_register_driver(&eee900freq_driver);

	if (!ret)
		printk(KERN_INFO MNAME
		       "CPU frequency scaling driver for the eeepc 900.\n");

	return ret;
}

static void __exit cpufreq_eee900_exit(void)
{
	cpufreq_unregister_driver(&eee900freq_driver);
	printk(KERN_INFO MNAME
	       "CPU frequency scaling driver for the eeepc 900 unregistered.\n");
}

module_init(cpufreq_eee900_init);
module_exit(cpufreq_eee900_exit);

MODULE_AUTHOR("Cristiano Prisciandaro <cristiano.p@solnet.ch>");
MODULE_DESCRIPTION("Frequency scaling driver for the eeepc 900.");
MODULE_LICENSE("GPL");
