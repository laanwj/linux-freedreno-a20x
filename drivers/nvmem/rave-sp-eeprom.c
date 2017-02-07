/*
 * zii-pic-eeprom.c - EEPROM driver for Zodiac Inflight Innovations
 * PIC MCU that is connected via dedicated UART port
 *
 * Copyright (C) 2017 Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * based on work by Andrey Vostrikov <andrey.vostrikov@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rave-sp.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>

enum {
	MAIN_EEPROM,
	DDS_EEPROM,
};

#define EEPROM_PAGE_SHIFT	5
#define EEPROM_PAGE_SIZE	(1 << EEPROM_PAGE_SHIFT)

#define CMD_MAIN_EEPROM		0xA4

#define CMD_DDS_EEPROM		0xA3

#define CMD_OLD_EEPROM		0x20

#define MAIN_EEPROM_SIZE	0x4000
#define DDS_EEPROM_SIZE_RDU1	0x2000
#define DDS_EEPROM_SIZE		0x4000

struct rave_sp_eeprom {
	struct rave_sp *zp;
	u8 cmd;
	u8 page_nr_bytes;
	struct nvmem_device *nvmem;
	struct mutex mutex;
};

static int rave_sp_eeprom_read_page(struct rave_sp_eeprom *zpe,
		unsigned int page, u8 *buf)
{
	u8 b[2 + EEPROM_PAGE_SIZE + 2];
	u8 p = 0;
	int ret;

	b[p++] = zpe->cmd;
	b[p++] = 0;
	b[p++] = 1;		/* read */
	b[p++] = page;
	if (zpe->page_nr_bytes > 1)
		b[p++] = page >> 8;

	ret = rave_sp_exec(zpe->zp,
			   b, p, 
			   b, 2 + EEPROM_PAGE_SIZE);
	if (ret)
		return ret;
	if (b[0] != 1)
		return -EPROTO;
	if (b[1] != 1)
		return -EIO;

	memcpy(buf, &b[2], EEPROM_PAGE_SIZE);
	return 0;
}

static int rave_sp_eeprom_write_page(struct rave_sp_eeprom *zpe,
		unsigned int page, u8 *buf)
{
	u8 b[3 + EEPROM_PAGE_SIZE + 2];
	u8 p = 0;
	int ret;

	b[p++] = zpe->cmd;
	b[p++] = 0;
	b[p++] = 0;		/* write */
	b[p++] = page;
	if (zpe->page_nr_bytes > 1)
		b[p++] = page >> 8;
	memcpy(&b[p], buf, EEPROM_PAGE_SIZE);

	ret = rave_sp_exec(zpe->zp,
			   b, p + EEPROM_PAGE_SIZE,
			   b, 2);
	if (ret)
		return ret;
	if (b[0] != 0)
		return -EPROTO;
	if (b[1] != 1)
		return -EIO;

	return 0;
}

static int rave_sp_eeprom_process(struct rave_sp_eeprom *zpe, bool write,
		unsigned int offset, u8 *buf, size_t bytes)
{
	unsigned int page = offset >> EEPROM_PAGE_SHIFT;
	u8 tmpbuf[EEPROM_PAGE_SIZE], tmpoffs, tmpsz;
	int ret;

	mutex_lock(&zpe->mutex);

	if (offset != page << EEPROM_PAGE_SHIFT || bytes < EEPROM_PAGE_SIZE) {
		ret = rave_sp_eeprom_read_page(zpe, page, tmpbuf);
		if (ret)
			goto out;
		tmpoffs = offset & (EEPROM_PAGE_SIZE - 1);
		tmpsz = min_t(size_t, EEPROM_PAGE_SIZE - tmpoffs, bytes);
		if (write) {
			memcpy(&tmpbuf[tmpoffs], buf, tmpsz);
			ret = rave_sp_eeprom_write_page(zpe, page, tmpbuf);
			if (ret)
				goto out;
		} else
			memcpy(buf, &tmpbuf[tmpoffs], tmpsz);
		buf += tmpsz;
		bytes -= tmpsz;
		page++;
	}

	while (bytes >= EEPROM_PAGE_SIZE) {
		if (write)
			ret = rave_sp_eeprom_write_page(zpe, page, buf);
		else
			ret = rave_sp_eeprom_read_page(zpe, page, buf);
		if (ret)
			goto out;
		buf += EEPROM_PAGE_SIZE;
		bytes -= EEPROM_PAGE_SIZE;
		page++;
	}

	if (bytes > 0) {
		ret = rave_sp_eeprom_read_page(zpe, page, tmpbuf);
		if (ret)
			goto out;
		if (write) {
			memcpy(tmpbuf, buf, bytes);
			ret = rave_sp_eeprom_write_page(zpe, page, tmpbuf);
			if (ret)
				goto out;
		} else
			memcpy(buf, tmpbuf, bytes);
	}

	ret = 0;
out:
	mutex_unlock(&zpe->mutex);
	return ret;
}

static int rave_sp_eeprom_reg_read(void *priv, unsigned int offset,
		void *val, size_t bytes)
{
	struct rave_sp_eeprom *zpe = priv;
	u8 *buf = val;

	return rave_sp_eeprom_process(zpe, false, offset, buf, bytes);
}

static int rave_sp_eeprom_reg_write(void *priv, unsigned int offset,
		void *val, size_t bytes)
{
	struct rave_sp_eeprom *zpe = priv;
	u8 *buf = val;

	return rave_sp_eeprom_process(zpe, true, offset, buf, bytes);
}

static const struct of_device_id rave_sp_eeprom_of_match[] = {
	{ .compatible = "zii,pic-main-eeprom", .data = (void *)MAIN_EEPROM },
	{ .compatible = "zii,pic-dds-eeprom", .data = (void *)DDS_EEPROM },
	{}
};

static int rave_sp_eeprom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rave_sp *zp = dev_get_drvdata(dev->parent);
	struct rave_sp_eeprom *zpe;
	const struct of_device_id *id;
	struct nvmem_config config;

	if (!zp)
		return -EINVAL;

	id = of_match_device(rave_sp_eeprom_of_match, dev);
	if (!id)
		return -ENODEV;

	zpe = devm_kzalloc(dev, sizeof(*zpe), GFP_KERNEL);
	if (!zpe)
		return -ENOMEM;
	platform_set_drvdata(pdev, zpe);

	zpe->zp = zp;
	mutex_init(&zpe->mutex);

	memset(&config, 0, sizeof(config));
	config.owner = THIS_MODULE;
	config.dev = dev;
	config.priv = zpe;
#if 0
	if (zp->hw_id < ZII_PIC_HW_ID_RDU1) {
		if (id->data == (void *)DDS_EEPROM)
			return -ENXIO;
		zpe->cmd = CMD_OLD_EEPROM;
		config.name = ZII_PIC_NAME_MAIN_EEPROM;
		config.size = MAIN_EEPROM_SIZE;
	} else if (id->data == (void *)MAIN_EEPROM) {
	/* if (zp->hw_id < RAVE_SP_HW_ID_RDU1) { */
	/* 	if (id->data == (void *)DDS_EEPROM) */
	/* 		return -ENXIO; */
	/* 	zpe->cmd = CMD_OLD_EEPROM; */
	/* 	config.name = RAVE_SP_NAME_MAIN_EEPROM; */
	/* 	config.size = MAIN_EEPROM_SIZE; */
	/* } else */ if (id->data == (void *)MAIN_EEPROM) {
		zpe->cmd = CMD_MAIN_EEPROM;
		config.name = ZII_PIC_NAME_MAIN_EEPROM;
		config.size = MAIN_EEPROM_SIZE;
	} else {
		zpe->cmd = CMD_DDS_EEPROM;
		config.name = ZII_PIC_NAME_DDS_EEPROM;
		config.size = zp->hw_id == ZII_PIC_HW_ID_RDU1 ?
			DDS_EEPROM_SIZE_RDU1 : DDS_EEPROM_SIZE;
	}
#endif
	config.reg_read = zii_pic_eeprom_reg_read;
	config.reg_write = zii_pic_eeprom_reg_write;

	config.word_size = 1;
	config.stride = 1;

	zpe->page_nr_bytes = config.size > 256 * EEPROM_PAGE_SIZE ? 2 : 1;

	zpe->nvmem = nvmem_register(&config);
	if (IS_ERR(zpe->nvmem))
		return PTR_ERR(zpe->nvmem);

	return 0;
}

static int rave_sp_eeprom_remove(struct platform_device *pdev)
{
	struct rave_sp_eeprom *zpe = platform_get_drvdata(pdev);

	return nvmem_unregister(zpe->nvmem);
}

static struct platform_driver rave_sp_eeprom_driver = {
	.probe = rave_sp_eeprom_probe,
	.remove = rave_sp_eeprom_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = rave_sp_eeprom_of_match,
	},
};
module_platform_driver(rave_sp_eeprom_driver);

MODULE_DEVICE_TABLE(of, rave_sp_eeprom_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC EEPROM driver");
