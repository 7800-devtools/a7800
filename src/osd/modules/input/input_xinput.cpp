// license:BSD-3-Clause
// copyright-holders:Brad Hughes
//============================================================
//
//  input_xinput.cpp - XInput API input support for Windows
//
//============================================================

#include "input_module.h"
#include "modules/osdmodule.h"

#if defined(OSD_WINDOWS)

// standard windows headers
#include <windows.h>

// XInput header
#include <xinput.h>

#undef interface

// MAME headers
#include "emu.h"

// MAMEOS headers
#include "winutil.h"
#include "winmain.h"

#include "input_common.h"
#include "input_windows.h"
#include "input_xinput.h"

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
#define XINPUT_LIBRARIES { "xinput1_4.dll", "xinput9_1_0.dll" }
#else
#define XINPUT_LIBRARIES { "xinput1_4.dll" }
#endif

int xinput_api_helper::initialize()
{
	m_xinput_dll = osd::dynamic_module::open(XINPUT_LIBRARIES);

	XInputGetState = m_xinput_dll->bind<xinput_get_state_fn>("XInputGetState");
	XInputGetCapabilities = m_xinput_dll->bind<xinput_get_caps_fn>("XInputGetCapabilities");

	if (!XInputGetState || !XInputGetCapabilities)
	{
		osd_printf_verbose("Could not find XInput. Please try to reinstall DirectX runtime package.\n");
		return -1;
	}

	return 0;
}

//============================================================
//  create_xinput_device
//============================================================

xinput_joystick_device * xinput_api_helper::create_xinput_device(running_machine &machine, UINT index, wininput_module &module)
{
	xinput_joystick_device *devinfo;

	XINPUT_CAPABILITIES caps = { 0 };
	if (FAILED(xinput_get_capabilities(index, 0, &caps)))
	{
		// If we can't get the capabilities skip this device
		return nullptr;
	}

	char device_name[16];
	snprintf(device_name, sizeof(device_name), "XInput Player %u", index + 1);

	// allocate the device object
	devinfo = module.devicelist()->create_device<xinput_joystick_device>(machine, device_name, device_name, module, shared_from_this());

	// Set the player ID
	devinfo->xinput_state.player_index = index;

	// Assign the caps we captured earlier
	devinfo->xinput_state.caps = caps;

	return devinfo;
}

//============================================================
//  xinput_joystick_device
//============================================================

xinput_joystick_device::xinput_joystick_device(running_machine &machine, const char *name, char const *id, input_module &module, std::shared_ptr<xinput_api_helper> helper)
	: device_info(machine, name, id, DEVICE_CLASS_JOYSTICK, module),
		gamepad({{0}}),
		xinput_state({0}),
		m_xinput_helper(helper),
		m_configured(false)
{
}

void xinput_joystick_device::poll()
{
	if (!m_configured)
		return;

	// poll the device first
	HRESULT result = m_xinput_helper->xinput_get_state(xinput_state.player_index, &xinput_state.xstate);

	// If we can't poll the device, skip
	if (FAILED(result))
		return;

	// Copy the XState into State
	// Start with the POV (DPAD)
	for (int povindex = 0; povindex < XINPUT_MAX_POV; povindex++)
	{
		int currentPov = xinput_pov_dir[povindex];
		gamepad.povs[povindex] = (xinput_state.xstate.Gamepad.wButtons & currentPov) ? 0xFF : 0;
	}

	// Now do the buttons
	for (int buttonindex = 0; buttonindex < XINPUT_MAX_BUTTONS; buttonindex++)
	{
		int currentButton = xinput_buttons[buttonindex];
		gamepad.buttons[buttonindex] = (xinput_state.xstate.Gamepad.wButtons & currentButton) ? 0xFF : 0;
	}

	// Now grab the axis values
	// Each of the thumbstick axis members is a signed value between -32768 and 32767 describing the position of the thumbstick
	// However, the Y axis values are inverted from what MAME expects, so negate the value
	gamepad.left_thumb_x = normalize_absolute_axis(xinput_state.xstate.Gamepad.sThumbLX, XINPUT_AXIS_MINVALUE, XINPUT_AXIS_MAXVALUE);
	gamepad.left_thumb_y = normalize_absolute_axis(-xinput_state.xstate.Gamepad.sThumbLY, XINPUT_AXIS_MINVALUE, XINPUT_AXIS_MAXVALUE);
	gamepad.right_thumb_x = normalize_absolute_axis(xinput_state.xstate.Gamepad.sThumbRX, XINPUT_AXIS_MINVALUE, XINPUT_AXIS_MAXVALUE);
	gamepad.right_thumb_y = normalize_absolute_axis(-xinput_state.xstate.Gamepad.sThumbRY, XINPUT_AXIS_MINVALUE, XINPUT_AXIS_MAXVALUE);

	// Now the triggers
	gamepad.left_trigger = normalize_absolute_axis(xinput_state.xstate.Gamepad.bLeftTrigger, 0, 255);
	gamepad.right_trigger = normalize_absolute_axis(xinput_state.xstate.Gamepad.bRightTrigger, 0, 255);
}

void xinput_joystick_device::reset()
{
	memset(&gamepad, 0, sizeof(gamepad));
}

void xinput_joystick_device::configure()
{
	std::lock_guard<std::mutex> scope_lock(m_device_lock);

	if (m_configured)
		return;

	// Add the axes
	for (int axisnum = 0; axisnum < XINPUT_MAX_AXIS; axisnum++)
	{
		device()->add_item(
			xinput_axis_name[axisnum],
			xinput_axis_ids[axisnum],
			generic_axis_get_state<LONG>,
			&gamepad.left_thumb_x + axisnum);
	}

	// Populate the POVs
	// For XBOX, we treat the DPAD as a hat switch
	for (int povnum = 0; povnum < XINPUT_MAX_POV; povnum++)
	{
		device()->add_item(
			xinput_pov_names[povnum],
			ITEM_ID_OTHER_SWITCH,
			generic_button_get_state<BYTE>,
			&gamepad.povs[povnum]);
	}

	// populate the buttons
	for (int butnum = 0; butnum < XINPUT_MAX_BUTTONS; butnum++)
	{
		device()->add_item(
			xinput_button_names[butnum],
			static_cast<input_item_id>(ITEM_ID_BUTTON1 + butnum),
			generic_button_get_state<BYTE>,
			&gamepad.buttons[butnum]);
	}

	device()->add_item(
		"Left Trigger",
		ITEM_ID_ZAXIS,
		generic_axis_get_state<LONG>,
		&gamepad.left_trigger);

	device()->add_item(
		"Right Trigger",
		ITEM_ID_RZAXIS,
		generic_axis_get_state<LONG>,
		&gamepad.right_trigger);

	m_configured = true;
}

//============================================================
//  xinput_joystick_module
//============================================================

class xinput_joystick_module : public wininput_module
{
private:
	std::shared_ptr<xinput_api_helper> m_xinput_helper;

public:
	xinput_joystick_module()
		: wininput_module(OSD_JOYSTICKINPUT_PROVIDER, "xinput"),
		m_xinput_helper(nullptr)
	{
	}

	int init(const osd_options &options) override
	{
		// Call the base
		int status = wininput_module::init(options);
		if (status != 0)
			return status;

		// Create and initialize our helper
		m_xinput_helper = std::make_shared<xinput_api_helper>();
		status = m_xinput_helper->initialize();
		if (status != 0)
		{
			osd_printf_error("xinput_joystick_module failed to get XInput interface! Error: %u\n", static_cast<unsigned int>(status));
			return -1;
		}

		return 0;
	}

protected:
	virtual void input_init(running_machine &machine) override
	{
		xinput_joystick_device *devinfo;

		// Loop through each gamepad to determine if they are connected
		for (UINT i = 0; i < XUSER_MAX_COUNT; i++)
		{
			XINPUT_STATE state = {0};

			if (m_xinput_helper->xinput_get_state(i, &state) == ERROR_SUCCESS)
			{
				// allocate and link in a new device
				devinfo = m_xinput_helper->create_xinput_device(machine, i, *this);
				if (devinfo == nullptr)
					continue;

				// Configure each gamepad to add buttons and Axes, etc.
				devinfo->configure();
			}
		}
	}
};

#else
MODULE_NOT_SUPPORTED(xinput_joystick_module, OSD_JOYSTICKINPUT_PROVIDER, "xinput")
#endif

MODULE_DEFINITION(JOYSTICKINPUT_XINPUT, xinput_joystick_module)
