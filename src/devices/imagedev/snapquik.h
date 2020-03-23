// license:BSD-3-Clause
// copyright-holders:Nathan Woods, Miodrag Milanovic
/*********************************************************************

    snapquik.h

    Snapshots and quickloads

*********************************************************************/

#ifndef MAME_DEVICES_IMAGEDEV_SNAPQUIK_H
#define MAME_DEVICES_IMAGEDEV_SNAPQUIK_H

#include "softlist_dev.h"

typedef delegate<image_init_result (device_image_interface &,const char *, int)> snapquick_load_delegate;

// ======================> snapshot_image_device
class snapshot_image_device :   public device_t,
								public device_image_interface
{
public:
	// construction/destruction
	snapshot_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~snapshot_image_device();

	static void static_set_interface(device_t &device, const char *_interface) { downcast<snapshot_image_device &>(device).m_interface = _interface; }

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const software_list_loader &get_software_list_loader() const override { return image_software_list_loader::instance(); }
	virtual iodevice_t image_type() const override { return IO_SNAPSHOT; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 0; }
	virtual const char *image_interface() const override { return m_interface; }
	virtual const char *file_extensions() const override { return m_file_extensions; }

	TIMER_CALLBACK_MEMBER(process_snapshot_or_quickload);
	void set_handler(snapquick_load_delegate load, const char *ext, seconds_t sec) { m_load = load; m_file_extensions = ext; m_delay_seconds = sec; };

protected:
	snapshot_image_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	snapquick_load_delegate m_load;                 /* loading function */
	const char *        m_file_extensions;      /* file extensions */
	const char *        m_interface;
	seconds_t           m_delay_seconds;        /* loading delay (seconds) */
	attoseconds_t       m_delay_attoseconds;    /* loading delay (attoseconds) */
	emu_timer           *m_timer;
};

// device type definition
DECLARE_DEVICE_TYPE(SNAPSHOT, snapshot_image_device)

// ======================> quickload_image_device

class quickload_image_device : public snapshot_image_device
{
public:
	// construction/destruction
	quickload_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual iodevice_t image_type() const override { return IO_QUICKLOAD; }
};

// device type definition
DECLARE_DEVICE_TYPE(QUICKLOAD, quickload_image_device)

/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/
#define SNAPSHOT_LOAD_MEMBER_NAME(_name)           snapshot_load_##_name
#define SNAPSHOT_LOAD_NAME(_class,_name)           _class::SNAPSHOT_LOAD_MEMBER_NAME(_name)
#define DECLARE_SNAPSHOT_LOAD_MEMBER(_name)        image_init_result SNAPSHOT_LOAD_MEMBER_NAME(_name)(device_image_interface &image, const char *file_type, int snapshot_size)
#define SNAPSHOT_LOAD_MEMBER(_class,_name)         image_init_result SNAPSHOT_LOAD_NAME(_class,_name)(device_image_interface &image, const char *file_type, int snapshot_size)
#define SNAPSHOT_LOAD_DELEGATE(_class,_name)       snapquick_load_delegate(&SNAPSHOT_LOAD_NAME(_class,_name), downcast<_class *>(device->owner()))

#define QUICKLOAD_LOAD_MEMBER_NAME(_name)           quickload_load##_name
#define QUICKLOAD_LOAD_NAME(_class,_name)           _class::QUICKLOAD_LOAD_MEMBER_NAME(_name)
#define DECLARE_QUICKLOAD_LOAD_MEMBER(_name)        image_init_result QUICKLOAD_LOAD_MEMBER_NAME(_name)(device_image_interface &image, const char *file_type, int quickload_size)
#define QUICKLOAD_LOAD_MEMBER(_class,_name)         image_init_result QUICKLOAD_LOAD_NAME(_class,_name)(device_image_interface &image, const char *file_type, int quickload_size)
#define QUICKLOAD_LOAD_DELEGATE(_class,_name)       snapquick_load_delegate(&QUICKLOAD_LOAD_NAME(_class,_name), downcast<_class *>(device->owner()))

#define MCFG_SNAPSHOT_ADD(_tag, _class, _load, _file_extensions, _delay) \
	MCFG_DEVICE_ADD(_tag, SNAPSHOT, 0) \
	static_cast<snapshot_image_device *>(device)->set_handler(SNAPSHOT_LOAD_DELEGATE(_class,_load), _file_extensions, _delay);

#define MCFG_SNAPSHOT_INTERFACE(_interface)                         \
	snapshot_image_device::static_set_interface(*device, _interface);

#define MCFG_QUICKLOAD_ADD(_tag, _class, _load, _file_extensions, _delay)   \
	MCFG_DEVICE_ADD(_tag, QUICKLOAD, 0) \
	static_cast<quickload_image_device *>(device)->set_handler(QUICKLOAD_LOAD_DELEGATE(_class,_load), _file_extensions, _delay);

#define MCFG_QUICKLOAD_INTERFACE(_interface)                         \
	quickload_image_device::static_set_interface(*device, _interface);

#endif // MAME_DEVICES_IMAGEDEV_SNAPQUIK_H
