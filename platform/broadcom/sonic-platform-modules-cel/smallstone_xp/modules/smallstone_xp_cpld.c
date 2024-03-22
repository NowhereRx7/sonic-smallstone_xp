/*
 * smallstone_xp_cpld.c - driver for Smallstone XP's CPLD
 *
 * Modified from:
 * dx010_cpld.c - driver for SeaStone's CPLD
 *
 * Copyright (C) 2023 Celestica Corp.
 * Pradchaya P <pphuchar@celestica.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <uapi/linux/stat.h>

#define DRIVER_NAME "smallstone_xp_cpld"

#define CPLD1_VERSION_ADDR 0x100
#define CPLD2_VERSION_ADDR 0x200

#define RESET0108   0x250
#define RESET0916   0x251
#define RESET1724   0x2d0
#define RESET2532   0x2d1

#define LPMOD0108   0x252
#define LPMOD0916   0x253
#define LPMOD1724   0x2d2
#define LPMOD2532   0x2d3

#define ABS0108     0x254
#define ABS0916     0x255
#define ABS1724     0x2d4
#define ABS2532     0x2d5

#define INT0108     0x256
#define INT0916     0x257
#define INT1724     0x2d6
#define INT2532     0x2d7

 //#define ABS_INT0108     0x260
 //#define ABS_INT0916     0x261
 //#define ABS_INT1724     0x2E0
 //#define ABS_INT2532     0x2E1

 //#define ABS_INT_MSK0108     0x262
 //#define ABS_INT_MSK0916     0x263
 //#define ABS_INT_MSK1724     0x2E2
 //#define ABS_INT_MSK2532     0x2E3

  //#define CPLD4_INT0          0x313
  //#define CPLD4_INT0_MSK      0x315

#define LENGTH_PORT_CPLD        32
#define PORT_BANK1_START        1
#define PORT_BANK1_END          16
#define PORT_BANK2_START        17
#define PORT_BANK2_END          32

#define PORT_ID_BANK1           0x210
#define PORT_ID_BANK2           0x290

#define OPCODE_ID_BANK1         0x211
#define OPCODE_ID_BANK2         0x291

#define DEVADDR_ID_BANK1        0x212
#define DEVADDR_ID_BANK2        0x292

#define CMDBYT_ID_BANK1         0x213
#define CMDBYT_ID_BANK2         0x293

#define WRITE_ID_BANK1          0x220
#define WRITE_ID_BANK2          0x2A0

#define READ_ID_BANK1           0x230
#define READ_ID_BANK2           0x2B0

#define SSRR_ID_BANK1           0x216
#define SSRR_ID_BANK2           0x296

#define SSRR_MASTER_ERR         0x80
#define SSRR_BUS_BUSY           0x40

#define I2C_BAUD_RATE_100K      0x40


struct smallstone_xp_i2c_data {
	int portid;
};

struct smallstone_xp_cpld_data {
	struct i2c_adapter* i2c_adapter[LENGTH_PORT_CPLD];
	struct mutex       cpld_lock;
	uint16_t           read_addr;
};

struct smallstone_xp_cpld_data* cpld_data;

static ssize_t getreg_store(struct device* dev, struct device_attribute* devattr,
	const char* buf, size_t count)
{

	uint16_t addr;
	char* last;

	addr = (uint16_t)strtoul(buf, &last, 16);
	if (addr == 0 && buf == last) {
		return -EINVAL;
	}
	cpld_data->read_addr = addr;
	return count;
}

static ssize_t getreg_show(struct device* dev, struct device_attribute* attr, char* buf)
{

	int len = 0;
	mutex_lock(&cpld_data->cpld_lock);
	len = sprintf(buf, "0x%2.2x\n", inb(cpld_data->read_addr));
	mutex_unlock(&cpld_data->cpld_lock);
	return len;
}

static ssize_t get_reset(struct device* dev, struct device_attribute* devattr, char* buf)
{
	unsigned long reset = 0;

	mutex_lock(&cpld_data->cpld_lock);

	reset =
		(inb(RESET2532) << 24) |
		(inb(RESET1724) << 16) |
		(inb(RESET0916) << 8) |
		inb(RESET0108);

	mutex_unlock(&cpld_data->cpld_lock);

	return sprintf(buf, "0x%8.8lx\n", reset & 0xffffffff);
}

static ssize_t setreg_store(struct device* dev, struct device_attribute* devattr,
	const char* buf, size_t count)
{

	uint16_t addr;
	uint8_t value;
	char* tok;
	char clone[count];
	char* pclone = clone;
	char* last;

	strcpy(clone, buf);

	mutex_lock(&cpld_data->cpld_lock);
	tok = strsep((char**)&pclone, " ");
	if (tok == NULL) {
		mutex_unlock(&cpld_data->cpld_lock);
		return -EINVAL;
	}
	addr = (uint16_t)strtoul(tok, &last, 16);
	if (addr == 0 && tok == last) {
		mutex_unlock(&cpld_data->cpld_lock);
		return -EINVAL;
	}

	tok = strsep((char**)&pclone, " ");
	if (tok == NULL) {
		mutex_unlock(&cpld_data->cpld_lock);
		return -EINVAL;
	}
	value = (uint8_t)strtoul(tok, &last, 16);
	if (value == 0 && tok == last) {
		mutex_unlock(&cpld_data->cpld_lock);
		return -EINVAL;
	}

	outb(value, addr);
	mutex_unlock(&cpld_data->cpld_lock);
	return count;
}

static ssize_t set_reset(struct device* dev, struct device_attribute* devattr,
	const char* buf, size_t count)
{
	unsigned long reset;
	int err;

	mutex_lock(&cpld_data->cpld_lock);

	err = kstrtoul(buf, 16, &reset);
	if (err)
	{
		mutex_unlock(&cpld_data->cpld_lock);
		return err;
	}

	outb((reset >> 0) & 0xFF, RESET0108);
	outb((reset >> 8) & 0xFF, RESET0916);
	outb((reset >> 16) & 0xFF, RESET1724);
	outb((reset >> 24) & 0xFF, RESET2532);

	mutex_unlock(&cpld_data->cpld_lock);

	return count;
}

static ssize_t get_lpmode(struct device* dev, struct device_attribute* devattr,
	char* buf)
{
	unsigned long lpmod = 0;

	mutex_lock(&cpld_data->cpld_lock);

	lpmod =
		(inb(LPMOD2532) << 24) |
		(inb(LPMOD1724) << 16) |
		(inb(LPMOD0916) << 8) |
		inb(LPMOD0108);

	mutex_unlock(&cpld_data->cpld_lock);

	return sprintf(buf, "0x%8.8lx\n", lpmod & 0xffffffff);
}

static ssize_t set_lpmode(struct device* dev, struct device_attribute* devattr,
	const char* buf, size_t count)
{
	unsigned long lpmod;
	int err;

	mutex_lock(&cpld_data->cpld_lock);

	err = kstrtoul(buf, 16, &lpmod);
	if (err)
	{
		mutex_unlock(&cpld_data->cpld_lock);
		return err;
	}

	outb((lpmod >> 0) & 0xFF, LPMOD0108);
	outb((lpmod >> 8) & 0xFF, LPMOD0916);
	outb((lpmod >> 16) & 0xFF, LPMOD1724);
	outb((lpmod >> 24) & 0xFF, LPMOD2532);

	mutex_unlock(&cpld_data->cpld_lock);

	return count;
}

static ssize_t get_modprs(struct device* dev, struct device_attribute* devattr,
	char* buf)
{
	unsigned long present;

	mutex_lock(&cpld_data->cpld_lock);

	present =
		(inb(ABS2532) << 24) |
		(inb(ABS1724) << 16) |
		(inb(ABS0916) << 8) |
		inb(ABS0108);

	mutex_unlock(&cpld_data->cpld_lock);

	return sprintf(buf, "0x%8.8lx\n", present & 0xffffffff);
}

static ssize_t get_modirq(struct device* dev, struct device_attribute* devattr,
	char* buf)
{
	unsigned long irq;

	mutex_lock(&cpld_data->cpld_lock);

	irq =
		(inb(INT2532) << 24) |
		(inb(INT1724) << 16) |
		(inb(INT0916) << 8) |
		inb(INT0108);

	mutex_unlock(&cpld_data->cpld_lock);

	return sprintf(buf, "0x%8.8lx\n", irq & 0xffffffff);
}

//static ssize_t get_modprs_irq(struct device* dev, struct device_attribute* devattr,
//	char* buf)
//{
//	unsigned long prs_int = 0;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	/* Clear interrupt source */
//	//inb(CPLD4_INT0);
//
//	prs_int =
//		(inb(ABS_INT2532) << 24) |
//		(inb(ABS_INT1724) << 16) |
//		(inb(ABS_INT0916) << 8) |
//		inb(ABS_INT0108);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return sprintf(buf, "0x%8.8lx\n", prs_int & 0xffffffff);
//}

//static ssize_t get_modprs_msk(struct device* dev, struct device_attribute* devattr,
//	char* buf)
//{
//	unsigned long prs_int_msk = 0;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	prs_int_msk =
//		(inb(ABS_INT_MSK2532) << 24) |
//		(inb(ABS_INT_MSK1724) << 16) |
//		(inb(ABS_INT_MSK0916) << 8) |
//		inb(ABS_INT_MSK0108);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return sprintf(buf, "0x%8.8lx\n", prs_int_msk & 0xffffffff);
//}

//static ssize_t set_modprs_msk(struct device* dev, struct device_attribute* devattr,
//	const char* buf, size_t count)
//{
//	unsigned long prs_int_msk;
//	int err;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	err = kstrtoul(buf, 16, &prs_int_msk);
//	if (err)
//	{
//		mutex_unlock(&cpld_data->cpld_lock);
//		return err;
//	}
//
//	outb((prs_int_msk >> 0) & 0xFF, ABS_INT_MSK0108);
//	outb((prs_int_msk >> 8) & 0xFF, ABS_INT_MSK0916);
//	outb((prs_int_msk >> 16) & 0xFF, ABS_INT_MSK1724);
//	outb((prs_int_msk >> 24) & 0xFF, ABS_INT_MSK2532);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return count;
//}

//static ssize_t get_cpld4_int0(struct device* dev, struct device_attribute* devattr,
//	char* buf)
//{
//	unsigned char int0 = 0;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	int0 = inb(CPLD4_INT0);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return sprintf(buf, "0x%2.2x\n", int0 & 0xff);
//}

//static ssize_t get_cpld4_int0_msk(struct device* dev, struct device_attribute* devattr,
//	char* buf)
//{
//	unsigned char int0_msk = 0;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	int0_msk = inb(CPLD4_INT0_MSK);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return sprintf(buf, "0x%2.2x\n", int0_msk & 0xff);
//}

//static ssize_t set_cpld4_int0_msk(struct device* dev, struct device_attribute* devattr,
//	const char* buf, size_t count)
//{
//	unsigned long int0_msk;
//	int err;
//
//	mutex_lock(&cpld_data->cpld_lock);
//
//	err = kstrtoul(buf, 16, &int0_msk);
//	if (err)
//	{
//		mutex_unlock(&cpld_data->cpld_lock);
//		return err;
//	}
//
//	outb(int0_msk & 0x3f, CPLD4_INT0_MSK);
//
//	mutex_unlock(&cpld_data->cpld_lock);
//
//	return count;
//}

static DEVICE_ATTR_RW(getreg);
static DEVICE_ATTR_WO(setreg);
static DEVICE_ATTR(qsfp_reset, S_IRUGO | S_IWUSR, get_reset, set_reset);
static DEVICE_ATTR(qsfp_lpmode, S_IRUGO | S_IWUSR, get_lpmode, set_lpmode);
static DEVICE_ATTR(qsfp_modprs, S_IRUGO, get_modprs, NULL);
static DEVICE_ATTR(qsfp_modirq, S_IRUGO, get_modirq, NULL);
// static DEVICE_ATTR(qsfp_modprs_irq, S_IRUGO, get_modprs_irq, NULL);
//static DEVICE_ATTR(qsfp_modprs_msk, S_IRUGO | S_IWUSR, get_modprs_msk, set_modprs_msk);
//static DEVICE_ATTR(cpld4_int0, S_IRUGO, get_cpld4_int0, NULL);
//static DEVICE_ATTR(cpld4_int0_msk, S_IRUGO | S_IWUSR, get_cpld4_int0_msk, set_cpld4_int0_msk);

static struct attribute* smallstone_xp_lpc_attrs[] = {
		&dev_attr_getreg.attr,
		&dev_attr_setreg.attr,
		&dev_attr_qsfp_reset.attr,
		&dev_attr_qsfp_lpmode.attr,
		&dev_attr_qsfp_modprs.attr,
		&dev_attr_qsfp_modirq.attr,
		//		&dev_attr_qsfp_modprs_irq.attr,
		//		&dev_attr_qsfp_modprs_msk.attr
		//		&dev_attr_cpld4_int0.attr,
		//		&dev_attr_cpld4_int0_msk.attr,
				NULL,
};

static struct attribute_group smallstone_xp_lpc_attr_grp = {
		.attrs = smallstone_xp_lpc_attrs,
};

static struct resource cel_smallstone_xp_lpc_resources[] = {
		{
				.flags = IORESOURCE_IO,
		},
};

static void cel_smallstone_xp_lpc_dev_release(struct device* dev)
{
	return;
}

static struct platform_device cel_smallstone_xp_lpc_dev = {
		.name = DRIVER_NAME,
		.id = -1,
		.num_resources = ARRAY_SIZE(cel_smallstone_xp_lpc_resources),
		.resource = cel_smallstone_xp_lpc_resources,
		.dev = {
			.release = cel_smallstone_xp_lpc_dev_release,
		}
};


/**
 * Read eeprom of QSFP device.
 * @param  a        i2c adapter.
 * @param  addr     address to read.
 * @param  new_data QSFP port number struct.
 * @param  cmd      i2c command.
 * @return          0 if not error, else the error code.
 */
 //static int i2c_read_eeprom(struct i2c_adapter* a, u16 addr,
 //	struct smallstone_xp_i2c_data* new_data, u8 cmd, union i2c_smbus_data* data) {
 //
 //	u32 reg;
 //	int ioBase = 0;
 //	char byte;
 //	short temp;
 //	short portid, opcode, devaddr, cmdbyte0, ssrr, writedata, readdata;
 //	__u16 word_data;
 //	int error = -EIO;
 //
 //	mutex_lock(&cpld_data->cpld_lock);
 //
 //	if ((new_data->portid >= PORT_BANK1_START) && (new_data->portid <= PORT_BANK1_END))
 //	{
 //		portid = PORT_ID_BANK1;
 //		opcode = OPCODE_ID_BANK1;
 //		devaddr = DEVADDR_ID_BANK1;
 //		cmdbyte0 = CMDBYT_ID_BANK1;
 //		ssrr = SSRR_ID_BANK1;
 //		writedata = WRITE_ID_BANK1;
 //		readdata = READ_ID_BANK1;
 //	}
 //	else if ((new_data->portid >= PORT_BANK2_START) && (new_data->portid <= PORT_BANK2_END)) {
 //		portid = PORT_ID_BANK2;
 //		opcode = OPCODE_ID_BANK2;
 //		devaddr = DEVADDR_ID_BANK2;
 //		cmdbyte0 = CMDBYT_ID_BANK2;
 //		ssrr = SSRR_ID_BANK2;
 //		writedata = WRITE_ID_BANK2;
 //		readdata = READ_ID_BANK2;
 //	}
 //	else {
 //		/* Invalid parameter! */
 //		error = -EINVAL;
 //		goto exit;
 //	}
 //
 //	while ((inb(ioBase + ssrr) & 0x40));
 //	if ((inb(ioBase + ssrr) & 0x80) == 0x80) {
 //		error = -EIO;
 //		/* Read error reset the port */
 //		outb(0x00, ioBase + ssrr);
 //		udelay(3000);
 //		outb(0x01, ioBase + ssrr);
 //		goto exit;
 //	}
 //
 //	byte = 0x40 + new_data->portid;
 //	reg = cmd;
 //	outb(byte, ioBase + portid);
 //	outb(reg, ioBase + cmdbyte0);
 //	byte = 33;
 //	outb(byte, ioBase + opcode);
 //	addr = addr << 1;
 //	addr |= 0x01;
 //	outb(addr, ioBase + devaddr);
 //	while ((inb(ioBase + ssrr) & 0x40))
 //	{
 //		udelay(100);
 //	}
 //
 //	if ((inb(ioBase + ssrr) & 0x80) == 0x80) {
 //		/* Read error reset the port */
 //		error = -EIO;
 //		outb(0x00, ioBase + ssrr);
 //		udelay(3000);
 //		outb(0x01, ioBase + ssrr);
 //		goto exit;
 //	}
 //
 //	temp = ioBase + readdata;
 //	word_data = inb(temp);
 //	word_data |= (inb(++temp) << 8);
 //
 //	mutex_unlock(&cpld_data->cpld_lock);
 //	data->word = word_data;
 //	return 0;
 //
 //exit:
 //	mutex_unlock(&cpld_data->cpld_lock);
 //	return error;
 //}


 /**
  * Read/Write eeprom of CPLD connected QSFP device.
  * @param  a        i2c adapter.
  * @param  addr     address to read.
  * @param  new_data QSFP port number struct.
  * @param  rw       read/write flag
  * @param  cmd      i2c command.
  * @param  size     access size
  * @return          0 if not error, else the error code.
  */
static int smallstone_xp_cpld_i2c_access(struct i2c_adapter* a, u16 addr,
	struct smallstone_xp_i2c_data* new_data, char rw,
	u8 cmd, int size, union i2c_smbus_data* data)
{
	u32 reg;
	int ioBase = 0;
	char byte;
	char data_len = 0;
	short temp;
	short portid, opcode, devaddr, cmdbyte0, ssrr, writedata, readdata;
	__u16 word_data;
	__u8  byte_data;
	int error = -EIO;

	mutex_lock(&cpld_data->cpld_lock);

	if ((new_data->portid >= PORT_BANK1_START) && (new_data->portid <= PORT_BANK1_END))
	{
		portid = PORT_ID_BANK1;
		opcode = OPCODE_ID_BANK1;
		devaddr = DEVADDR_ID_BANK1;
		cmdbyte0 = CMDBYT_ID_BANK1;
		ssrr = SSRR_ID_BANK1;
		writedata = WRITE_ID_BANK1;
		readdata = READ_ID_BANK1;
	}
	else if ((new_data->portid >= PORT_BANK2_START) && (new_data->portid <= PORT_BANK2_END)) {
		portid = PORT_ID_BANK2;
		opcode = OPCODE_ID_BANK2;
		devaddr = DEVADDR_ID_BANK2;
		cmdbyte0 = CMDBYT_ID_BANK2;
		ssrr = SSRR_ID_BANK2;
		writedata = WRITE_ID_BANK2;
		readdata = READ_ID_BANK2;
	}
	else {
		/* Invalid parameter! */
		error = -EINVAL;
		goto exit;
	}

	if (size == I2C_SMBUS_BYTE || size == I2C_SMBUS_BYTE_DATA)
		data_len = 1;
	else if (size == I2C_SMBUS_WORD_DATA)
		data_len = 2;
	else {
		error = -EINVAL;
		goto exit;
	}

	while ((inb(ioBase + ssrr) & SSRR_BUS_BUSY));
	if ((inb(ioBase + ssrr) & SSRR_MASTER_ERR) == SSRR_MASTER_ERR) {
		error = -EIO;
		/* Read error reset the port */
		outb(0x00, ioBase + ssrr);
		udelay(3000);
		outb(0x01, ioBase + ssrr);
		goto exit;
	}

	byte = I2C_BAUD_RATE_100K + new_data->portid;
	reg = cmd;
	outb(byte, ioBase + portid);
	outb(reg, ioBase + cmdbyte0);
	byte = (data_len << 4) | 0x1;
	outb(byte, ioBase + opcode);
	addr = addr << 1;
	if (rw == I2C_SMBUS_READ)
	{
		addr |= 0x01;
		outb(addr, ioBase + devaddr);
		while ((inb(ioBase + ssrr) & SSRR_BUS_BUSY))
		{
			udelay(100);
		}

		if ((inb(ioBase + ssrr) & SSRR_MASTER_ERR) == SSRR_MASTER_ERR) {
			/* Read error reset the port */
			error = -EIO;
			outb(0x00, ioBase + ssrr);
			udelay(3000);
			outb(0x01, ioBase + ssrr);
			goto exit;
		}

		temp = ioBase + readdata;
		if (data_len == 1)
		{
			byte_data = inb(temp);
			data->byte = byte_data;
		}
		else if (data_len == 2)
		{
			word_data = inb(temp);
			word_data |= (inb(++temp) << 8);
			data->word = word_data;
		}
	}
	else // do i2c write
	{
		temp = ioBase + writedata;
		if (data_len == 1)
		{
			byte_data = data->byte;
			outb(byte_data, temp);
		}
		else if (data_len == 2)
		{
			word_data = data->word;
			outb((word_data & 0xff), temp);
			outb((word_data >> 4), (++temp));
		}
		// write dev addr
		outb(addr, ioBase + devaddr);

		// check bus access status
		while ((inb(ioBase + ssrr) & SSRR_BUS_BUSY))
		{
			udelay(100);
		}

		if ((inb(ioBase + ssrr) & SSRR_MASTER_ERR) == SSRR_MASTER_ERR) {
			/* Read error reset the port */
			error = -EIO;
			outb(0x00, ioBase + ssrr);
			udelay(3000);
			outb(0x01, ioBase + ssrr);
			goto exit;
		}
	}

	mutex_unlock(&cpld_data->cpld_lock);
	return 0;

exit:
	mutex_unlock(&cpld_data->cpld_lock);
	return error;
}


static int smallstone_xp_i2c_access(struct i2c_adapter* a, u16 addr,
	unsigned short flags, char rw, u8 cmd,
	int size, union i2c_smbus_data* data)
{

	int error = 0;

	struct smallstone_xp_i2c_data* new_data;

	/* Write the command register */
	new_data = i2c_get_adapdata(a);

	/* Map the size to what the chip understands */
	switch (size) {
	case I2C_SMBUS_BYTE:
	case I2C_SMBUS_BYTE_DATA:
	case I2C_SMBUS_WORD_DATA:
		if (0 == smallstone_xp_cpld_i2c_access(a, addr, new_data, rw, cmd, size, data)) {
			error = 0;
		}
		else {
			error = -EIO;
		}
		break;
	default:
		dev_warn(&a->dev, "Unsupported transaction %d\n", size);
		error = -EOPNOTSUPP;
		goto Done;
	}

Done:
	return error;
}

static u32 smallstone_xp_i2c_func(struct i2c_adapter* a)
{
	return I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm smallstone_xp_i2c_algorithm = {
		.smbus_xfer = smallstone_xp_i2c_access,
		.functionality = smallstone_xp_i2c_func,
};

static struct i2c_adapter* cel_smallstone_xp_i2c_init(struct platform_device* pdev, int portid)
{
	int error;

	struct i2c_adapter* new_adapter;
	struct smallstone_xp_i2c_data* new_data;

	new_adapter = kzalloc(sizeof(*new_adapter), GFP_KERNEL);
	if (!new_adapter)
		return NULL;

	new_adapter->dev.parent = &pdev->dev;
	new_adapter->owner = THIS_MODULE;
	new_adapter->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	new_adapter->algo = &smallstone_xp_i2c_algorithm;

	snprintf(new_adapter->name, sizeof(new_adapter->name),
		"SMBus Smallstone XP i2c Adapter portid@%04x", portid);

	new_data = kzalloc(sizeof(*new_data), GFP_KERNEL);
	if (!new_data)
		return NULL;

	new_data->portid = portid;

	i2c_set_adapdata(new_adapter, new_data);

	error = i2c_add_adapter(new_adapter);
	if (error)
		return NULL;

	return new_adapter;
};

static int cel_smallstone_xp_lpc_drv_probe(struct platform_device* pdev)
{
	struct resource* res;
	int ret = 0;
	int portid_count;

	cpld_data = devm_kzalloc(&pdev->dev, sizeof(struct smallstone_xp_cpld_data),
		GFP_KERNEL);
	if (!cpld_data)
		return -ENOMEM;

	mutex_init(&cpld_data->cpld_lock);
	cpld_data->read_addr = CPLD1_VERSION_ADDR;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (unlikely(!res)) {
		printk(KERN_ERR " Specified Resource Not Available...\n");
		return -1;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &smallstone_xp_lpc_attr_grp);
	if (ret) {
		printk(KERN_ERR "Cannot create sysfs\n");
	}

	for (portid_count = 1; portid_count <= LENGTH_PORT_CPLD; portid_count++)
		cpld_data->i2c_adapter[portid_count - 1] =
		cel_smallstone_xp_i2c_init(pdev, portid_count);

	/* Enable INT0 interrupt register */
	//outb(inb(CPLD4_INT0_MSK) & 0xf8, CPLD4_INT0_MSK);

	/* Enable modprs interrupt register */
	//outb(0, ABS_INT_MSK0108);
	//outb(0, ABS_INT_MSK0916);
	//outb(0, ABS_INT_MSK1724);
	//outb(0, ABS_INT_MSK2532);

	return 0;
}

static int cel_smallstone_xp_lpc_drv_remove(struct platform_device* pdev)
{
	int portid_count;

	sysfs_remove_group(&pdev->dev.kobj, &smallstone_xp_lpc_attr_grp);

	for (portid_count = 1; portid_count <= LENGTH_PORT_CPLD; portid_count++)
		i2c_del_adapter(cpld_data->i2c_adapter[portid_count - 1]);

	return 0;
}

static struct platform_driver cel_smallstone_xp_lpc_drv = {
		.probe = cel_smallstone_xp_lpc_drv_probe,
		.remove = __exit_p(cel_smallstone_xp_lpc_drv_remove),
		.driver = {
		.name = DRIVER_NAME,
		},
};

int cel_smallstone_xp_lpc_init(void)
{
	platform_device_register(&cel_smallstone_xp_lpc_dev);
	platform_driver_register(&cel_smallstone_xp_lpc_drv);

	return 0;
}

void cel_smallstone_xp_lpc_exit(void)
{
	platform_driver_unregister(&cel_smallstone_xp_lpc_drv);
	platform_device_unregister(&cel_smallstone_xp_lpc_dev);
}

module_init(cel_smallstone_xp_lpc_init);
module_exit(cel_smallstone_xp_lpc_exit);

MODULE_AUTHOR("NowhereRx7 <nowhererx7@yahoo.com>");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("Celestica Smallstone XP CPLD Driver");
MODULE_LICENSE("GPL");