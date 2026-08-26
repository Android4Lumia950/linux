// SPDX-License-Identifier: GPL-2.0-only
/*
 * Talkman lab convenience: octagon camsnap-key is PM8994 GPIO 4,
 * KEY_CAMERA (msm8994-msft-lumia-octagon.dtsi). On a living mainline
 * boot, a full camera-button press emergency_restarts with a unique
 * printk so pstore is not read as a crash.
 *
 * BCD/lk1st still own the button before Linux. After a panic the
 * IRQ does not run; Power long-press is still required.
 * Do not reboot on camfocus (octagon maps that to KEY_VOLUMEUP).
 */
#include <linux/input.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>

static void talkman_camera_event(struct input_handle *handle,
				 unsigned int type, unsigned int code, int value)
{
	if (type != EV_KEY || code != KEY_CAMERA || value != 1)
		return;

	pr_emerg("talkman: camera button reboot (not a crash)\n");
	emergency_restart();
}

static int talkman_camera_connect(struct input_handler *handler,
				  struct input_dev *dev,
				  const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "talkman-camera-reboot";

	error = input_register_handle(handle);
	if (error)
		goto err_free;

	error = input_open_device(handle);
	if (error)
		goto err_unregister;

	pr_info("talkman: camera snapshot reboot armed on %s\n", dev->name);
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return error;
}

static void talkman_camera_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id talkman_camera_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_CAMERA)] = BIT_MASK(KEY_CAMERA) },
	},
	{ },
};

static struct input_handler talkman_camera_handler = {
	.event = talkman_camera_event,
	.connect = talkman_camera_connect,
	.disconnect = talkman_camera_disconnect,
	.name = "talkman-camera-reboot",
	.id_table = talkman_camera_ids,
};

static int __init talkman_camera_init(void)
{
	return input_register_handler(&talkman_camera_handler);
}
late_initcall(talkman_camera_init);
