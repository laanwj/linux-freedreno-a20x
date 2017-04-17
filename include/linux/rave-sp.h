#ifndef _LINUX_RAVE_SP_H_
#define _LINUX_RAVE_SP_H_

enum rave_sp_command {
	RAVE_SP_CMD_GET_FIRMWARE_VERSION	= 0x20,
	RAVE_SP_CMD_GET_BOOTLOADER_VERSION	= 0x21,
	RAVE_SP_CMD_BOOT_SOURCE			= 0x26,
	RAVE_SP_CMD_GET_BOARD_COPPER_REV	= 0x2B,
	RAVE_SP_CMD_GET_GPIO_STATE		= 0x2F,

	RAVE_SP_CMD_STATUS			= 0xA0,
	RAVE_SP_CMD_SW_WDT			= 0xA1,
	RAVE_SP_CMD_PET_WDT			= 0xA2,
	RAVE_SP_CMD_RESET			= 0xA7,
	RAVE_SP_CMD_RESET_REASON		= 0xA8,

	RAVE_SP_CMD_REQ_COPPER_REV		= 0xB6,
	RAVE_SP_CMD_GET_I2C_DEVICE_STATUS	= 0xBA,
	RAVE_SP_CMD_GET_SP_SILICON_REV		= 0xB9,
	RAVE_SP_CMD_CONTROL_EVENTS		= 0xBB,

	RAVE_SP_EVNT_BASE			= 0xE0,
};

struct rave_sp;

static inline unsigned long rave_sp_action(u8 event, u8 value)
{
	return ((unsigned long)value << 8) | event;
}

static inline u8 rave_sp_action_get_event(unsigned long action)
{
	return action & 0xff;
}

static inline u8 rave_sp_action_get_value(unsigned long action)
{
	return (action >> 8) & 0xff;
}

int rave_sp_exec(struct rave_sp *sp,
		 void *__data,  size_t data_size,
		 void *reply_data, size_t reply_data_size);
int devm_rave_sp_register_event_notifier(struct device *dev,
				     struct notifier_block *nb);

#define COMPATIBLE_RAVE_SP_NIU		"zii,rave-sp-niu"
#define COMPATIBLE_RAVE_SP_MEZZ		"zii,rave-sp-mezz"
#define COMPATIBLE_RAVE_SP_ESB		"zii,rave-sp-esb"
#define COMPATIBLE_RAVE_SP_RDU1		"zii,rave-sp-rdu1"
#define COMPATIBLE_RAVE_SP_RDU2		"zii,rave-sp-rdu2"

#endif /* _LINUX_RAVE_SP_H_ */
