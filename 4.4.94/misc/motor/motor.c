// SPDX-License-Identifier: GPL-2.0+
/*
 * motor.c - Ingenic motor driver (T41 port)
 *
 * Based on 3.10 driver by Thingino Project / Ingenic Semiconductor
 * Ported to 4.4 kernel for T41 SoC
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/file.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mempolicy.h>
#include <linux/interrupt.h>
#include <linux/mfd/ingenic-tcu.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#ifdef CONFIG_SOC_T31
#include <dt-bindings/interrupt-controller/t31-irq.h>
#endif
#ifdef CONFIG_SOC_T40
#include <dt-bindings/interrupt-controller/t40-irq.h>
#endif

#include <soc/base.h>
#include <soc/extal.h>
#include <soc/gpio.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include "motor.h"

#define JZ_MOTOR_DRIVER_VERSION "H20171206a-t41"

extern int jzgpio_ctrl_pull(enum gpio_port port, int enable_pull, unsigned long pins);

/* Pan motor phase GPIOs */
static int hst1 = -1;
module_param(hst1, int, S_IRUGO);
MODULE_PARM_DESC(hst1, "Pan motor Phase A GPIO");
static int hst2 = -1;
module_param(hst2, int, S_IRUGO);
MODULE_PARM_DESC(hst2, "Pan motor Phase B GPIO");
static int hst3 = -1;
module_param(hst3, int, S_IRUGO);
MODULE_PARM_DESC(hst3, "Pan motor Phase C GPIO");
static int hst4 = -1;
module_param(hst4, int, S_IRUGO);
MODULE_PARM_DESC(hst4, "Pan motor Phase D GPIO");

/* Tilt motor phase GPIOs */
static int vst1 = -1;
module_param(vst1, int, S_IRUGO);
MODULE_PARM_DESC(vst1, "Tilt motor Phase A GPIO");
static int vst2 = -1;
module_param(vst2, int, S_IRUGO);
MODULE_PARM_DESC(vst2, "Tilt motor Phase B GPIO");
static int vst3 = -1;
module_param(vst3, int, S_IRUGO);
MODULE_PARM_DESC(vst3, "Tilt motor Phase C GPIO");
static int vst4 = -1;
module_param(vst4, int, S_IRUGO);
MODULE_PARM_DESC(vst4, "Tilt motor Phase D GPIO");

/* Max steps */
static int hmaxstep = 3700;
module_param(hmaxstep, int, S_IRUGO);
MODULE_PARM_DESC(hmaxstep, "Pan motor max steps");
static int vmaxstep = 1000;
module_param(vmaxstep, int, S_IRUGO);
MODULE_PARM_DESC(vmaxstep, "Tilt motor max steps");

/* Endstop GPIOs (-1 = software endstops) */
static int hmin = -1;
module_param(hmin, int, S_IRUGO);
MODULE_PARM_DESC(hmin, "Pan motor min endstop GPIO (-1 = disabled)");
static int hmax = -1;
module_param(hmax, int, S_IRUGO);
MODULE_PARM_DESC(hmax, "Pan motor max endstop GPIO (-1 = disabled)");
static int vmin = -1;
module_param(vmin, int, S_IRUGO);
MODULE_PARM_DESC(vmin, "Tilt motor min endstop GPIO (-1 = disabled)");
static int vmax = -1;
module_param(vmax, int, S_IRUGO);
MODULE_PARM_DESC(vmax, "Tilt motor max endstop GPIO (-1 = disabled)");

/* Misc */
static int motor_switch_gpio = -1;
module_param(motor_switch_gpio, int, S_IRUGO);
MODULE_PARM_DESC(motor_switch_gpio, "Motor direction switch GPIO (-1 = disabled)");
static int invert_gpio_dir = 0;
module_param(invert_gpio_dir, int, S_IRUGO);
MODULE_PARM_DESC(invert_gpio_dir, "Invert motor GPIO values (0/1)");
static int invert_direction_polarity = 1;
module_param(invert_direction_polarity, int, S_IRUGO);
MODULE_PARM_DESC(invert_direction_polarity, "Invert motor_switch_gpio polarity (0/1)");
static unsigned int hmotor2vmotor = 1;
module_param(hmotor2vmotor, int, S_IRUGO);
MODULE_PARM_DESC(hmotor2vmotor, "Pan/tilt step ratio");
static int motor_bind_channel = 2;
module_param(motor_bind_channel, int, S_IRUGO);
MODULE_PARM_DESC(motor_bind_channel, "TCU channel to bind (default 2)");

/* Platform data */
struct motor_platform_data motors_pdata[HAS_MOTOR_CNT] = {
	{
		.name              = "Pan motor",
		.motor_st1_gpio    = -1,
		.motor_st2_gpio    = -1,
		.motor_st3_gpio    = -1,
		.motor_st4_gpio    = -1,
		.motor_min_gpio    = -1,
		.motor_max_gpio    = -1,
		.motor_switch_gpio = -1,
	},
	{
		.name              = "Tilt motor",
		.motor_st1_gpio    = -1,
		.motor_st2_gpio    = -1,
		.motor_st3_gpio    = -1,
		.motor_st4_gpio    = -1,
		.motor_min_gpio    = -1,
		.motor_max_gpio    = -1,
		.motor_switch_gpio = -1,
	},
};

/* Step sequence */
static unsigned char step_8[8] = {
	0x08, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01, 0x09
};

/* Soft-start ramp table */
static char skip_move_mode[4][4] = {
	{2, 0, 0, 0},
	{3, 2, 0, 0},
	{4, 3, 2, 0},
	{4, 3, 2, 1}
};

static inline void calc_slow_mode(struct motor_device *mdev, unsigned int steps)
{
	int index = steps / 10;
	index = index > 3 ? 3 : index;
	mdev->skip_mode = skip_move_mode[index];
}

static inline int whether_move_func(struct motor_device *mdev, unsigned int remainder)
{
	if (remainder == 0)
		return 0;
	remainder = remainder / 10;
	remainder = remainder > 3 ? 3 : remainder;
	if (mdev->counter % mdev->skip_mode[remainder] == 0)
		return 1;
	return 0;
}

/* GPIO helpers */
static void motor_set_direction(struct motor_device *mdev, int move_direction)
{
	if (motor_switch_gpio != -1) {
		int gpio_value = (move_direction == MOTOR_MOVE_RIGHT_UP) ? 1 : 0;
		if (invert_direction_polarity)
			gpio_value = !gpio_value;
		gpio_direction_output(motor_switch_gpio, gpio_value);
	}
}

static void motor_power_off(struct motor_device *mdev)
{
	int index, value;
	struct motor_driver *motor;
	value = (invert_gpio_dir & 0x1);
	for (index = 0; index < HAS_MOTOR_CNT; index++) {
		motor = &mdev->motors[index];
		if (motor->pdata->motor_st1_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st1_gpio, value);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st2_gpio, value);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st3_gpio, value);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st4_gpio, value);
	}
}

static void motor_power_on(struct motor_device *mdev)
{
	int index, value;
	struct motor_driver *motor;
	value = ((0 ^ invert_gpio_dir) & 0x1);
	for (index = 0; index < HAS_MOTOR_CNT; index++) {
		motor = &mdev->motors[index];
		if (motor->pdata->motor_st1_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st1_gpio, value);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st2_gpio, value);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st3_gpio, value);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st4_gpio, value);
	}
}

static void motor_set_default(struct motor_device *mdev)
{
	int index;
	mdev->dev_state = MOTOR_OPS_STOP;
	for (index = 0; index < HAS_MOTOR_CNT; index++)
		mdev->motors[index].state = MOTOR_OPS_STOP;
	motor_power_off(mdev);
}

/* Step output with XOR inversion (active-low coil drive) */
static void motor_move_step(struct motor_device *mdev, int index)
{
	int step, value;
	struct motor_driver *motor = &mdev->motors[index];

	if (motor->state != MOTOR_OPS_STOP) {
		step  = motor->cur_steps % 8;
		step  = step < 0 ? step + 8 : step;
		value = (step_8[step] ^ 0xff);  /* XOR: active-low */

		motor_set_direction(mdev, (index == HORIZONTAL_MOTOR) ?
				    MOTOR_MOVE_RIGHT_UP : MOTOR_MOVE_LEFT_DOWN);

		if (motor->pdata->motor_st1_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st1_gpio, value & 0x8);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st2_gpio, value & 0x4);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st3_gpio, value & 0x2);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st4_gpio, value & 0x1);
	} else {
		value = ((0 ^ invert_gpio_dir) & 0x1);
		if (motor->pdata->motor_st1_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st1_gpio, value);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st2_gpio, value);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st3_gpio, value);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_direction_output(motor->pdata->motor_st4_gpio, value);
	}

	if (motor->state == MOTOR_OPS_RESET)
		motor->total_steps++;
}

/* Software endstop handlers */
static void move_to_min_pose_ops(struct motor_driver *motor)
{
	if (motor->state == MOTOR_OPS_RESET) {
		if (motor->move_dir == MOTOR_MOVE_LEFT_DOWN)
			motor->state = MOTOR_OPS_STOP;
	} else {
		motor->move_dir = MOTOR_MOVE_RIGHT_UP;
	}
	motor->cur_steps = 0;
}

static void move_to_max_pose_ops(struct motor_driver *motor, int index)
{
	if (motor->state == MOTOR_OPS_RESET) {
		motor->state = MOTOR_OPS_STOP;
		motor->max_steps = (index == HORIZONTAL_MOTOR) ? hmaxstep : vmaxstep;
		complete(&motor->reset_completion);
		motor->move_dir = MOTOR_MOVE_LEFT_DOWN;
	} else if (motor->state == MOTOR_OPS_NORMAL) {
		if (motor->move_dir == MOTOR_MOVE_RIGHT_UP)
			motor->state = MOTOR_OPS_STOP;
	} else {
		motor->move_dir = MOTOR_MOVE_LEFT_DOWN;
	}
	motor->cur_steps = motor->max_steps;
}

/* TCU interrupt handler */
static irqreturn_t jz_timer_interrupt(int irq, void *dev_id)
{
	struct motor_device *mdev = dev_id;
	struct motor_move   *dst  = &mdev->dst_move;
	struct motor_move   *cur  = &mdev->cur_move;
	struct motor_driver *motors = mdev->motors;

	if (motors[HORIZONTAL_MOTOR].state == MOTOR_OPS_STOP &&
	    motors[VERTICAL_MOTOR].state   == MOTOR_OPS_STOP) {
		mdev->dev_state = MOTOR_OPS_STOP;
		motor_move_step(mdev, HORIZONTAL_MOTOR);
		motor_move_step(mdev, VERTICAL_MOTOR);
		if (mdev->wait_stop) {
			mdev->wait_stop = 0;
			complete(&mdev->stop_completion);
		}
		return IRQ_HANDLED;
	}

	/* Software endstop checks */
	if (motors[HORIZONTAL_MOTOR].cur_steps <= 0)
		move_to_min_pose_ops(&motors[HORIZONTAL_MOTOR]);
	if (motors[HORIZONTAL_MOTOR].cur_steps >= motors[HORIZONTAL_MOTOR].max_steps)
		move_to_max_pose_ops(&motors[HORIZONTAL_MOTOR], HORIZONTAL_MOTOR);
	if (motors[VERTICAL_MOTOR].cur_steps <= 0)
		move_to_min_pose_ops(&motors[VERTICAL_MOTOR]);
	if (motors[VERTICAL_MOTOR].cur_steps >= motors[VERTICAL_MOTOR].max_steps)
		move_to_max_pose_ops(&motors[VERTICAL_MOTOR], VERTICAL_MOTOR);

	if (mdev->dev_state == MOTOR_OPS_CRUISE) {
		mdev->counter++;
		motors[HORIZONTAL_MOTOR].cur_steps += motors[HORIZONTAL_MOTOR].move_dir;
		if (mdev->counter % hmotor2vmotor == 0)
			motors[VERTICAL_MOTOR].cur_steps += motors[VERTICAL_MOTOR].move_dir;
		motor_move_step(mdev, HORIZONTAL_MOTOR);
		motor_move_step(mdev, VERTICAL_MOTOR);

	} else if (mdev->dev_state == MOTOR_OPS_RESET) {
		if (motors[HORIZONTAL_MOTOR].state != MOTOR_OPS_STOP) {
			motors[HORIZONTAL_MOTOR].cur_steps += motors[HORIZONTAL_MOTOR].move_dir;
			motor_move_step(mdev, HORIZONTAL_MOTOR);
			cur->one.x++;
		}
		if (motors[VERTICAL_MOTOR].state != MOTOR_OPS_STOP) {
			motors[VERTICAL_MOTOR].cur_steps += motors[VERTICAL_MOTOR].move_dir;
			motor_move_step(mdev, VERTICAL_MOTOR);
			cur->one.y++;
		}

	} else {
		mdev->counter++;

		if (cur->one.x < dst->one.x &&
		    motors[HORIZONTAL_MOTOR].state != MOTOR_OPS_STOP) {
			if (whether_move_func(mdev, dst->one.x - cur->one.x)) {
				motors[HORIZONTAL_MOTOR].cur_steps += motors[HORIZONTAL_MOTOR].move_dir;
				motor_move_step(mdev, HORIZONTAL_MOTOR);
				cur->one.x++;
			}
		} else {
			motors[HORIZONTAL_MOTOR].state = MOTOR_OPS_STOP;
		}

		if (cur->one.y < dst->one.y &&
		    motors[VERTICAL_MOTOR].state != MOTOR_OPS_STOP) {
			if (mdev->counter % hmotor2vmotor == 0) {
				motors[VERTICAL_MOTOR].cur_steps += motors[VERTICAL_MOTOR].move_dir;
				cur->one.y++;
				motor_move_step(mdev, VERTICAL_MOTOR);
			}
		} else {
			motors[VERTICAL_MOTOR].state = MOTOR_OPS_STOP;
		}
	}

	return IRQ_HANDLED;
}

/* Motor operations */
static long motor_ops_move(struct motor_device *mdev, int x, int y)
{
	struct motor_driver *motors = mdev->motors;
	unsigned long flags;
	int x_dir, y_dir, x1, y1;

	if (x > 0) {
		if (motors[HORIZONTAL_MOTOR].cur_steps >= motors[HORIZONTAL_MOTOR].max_steps)
			x = 0;
	} else {
		if (motors[HORIZONTAL_MOTOR].cur_steps <= 0)
			x = 0;
	}
	if (y > 0) {
		if (motors[VERTICAL_MOTOR].cur_steps >= motors[VERTICAL_MOTOR].max_steps)
			y = 0;
	} else {
		if (motors[VERTICAL_MOTOR].cur_steps <= 0)
			y = 0;
	}

	x_dir = x > 0 ? MOTOR_MOVE_RIGHT_UP : MOTOR_MOVE_LEFT_DOWN;
	y_dir = y > 0 ? MOTOR_MOVE_RIGHT_UP : MOTOR_MOVE_LEFT_DOWN;
	x1    = x < 0 ? -x : x;
	y1    = y < 0 ? -y : y;

	if ((x1 + y1) == 0)
		return 0;

	motor_power_on(mdev);

	mutex_lock(&mdev->dev_mutex);
	spin_lock_irqsave(&mdev->slock, flags);

	calc_slow_mode(mdev, x1);
	mdev->counter        = 0;
	mdev->dev_state      = MOTOR_OPS_NORMAL;
	mdev->dst_move.one.x = x1;
	mdev->dst_move.one.y = y1;
	mdev->cur_move.one.x = 0;
	mdev->cur_move.one.y = 0;

	motors[HORIZONTAL_MOTOR].state    = MOTOR_OPS_NORMAL;
	motors[HORIZONTAL_MOTOR].move_dir = x_dir;
	motors[VERTICAL_MOTOR].state      = MOTOR_OPS_NORMAL;
	motors[VERTICAL_MOTOR].move_dir   = y_dir;

	spin_unlock_irqrestore(&mdev->slock, flags);
	mutex_unlock(&mdev->dev_mutex);

	ingenic_tcu_counter_begin(mdev->tcu);
	return 0;
}

static void motor_ops_stop(struct motor_device *mdev)
{
	long ret;
	unsigned long flags;
	unsigned int remainder;
	struct motor_driver *motors = mdev->motors;
	struct motor_move   *dst    = &mdev->dst_move;
	struct motor_move   *cur    = &mdev->cur_move;

	if (mdev->dev_state == MOTOR_OPS_STOP)
		return;

	mutex_lock(&mdev->dev_mutex);
	spin_lock_irqsave(&mdev->slock, flags);

	if (mdev->dev_state == MOTOR_OPS_NORMAL) {
		remainder = dst->one.x - cur->one.x;
		if (remainder > 30) { dst->one.x = 29; cur->one.x = 0; }
		remainder = dst->one.y - cur->one.y;
		if (remainder > 8)  { dst->one.y = 6;  cur->one.y = 0; }
	}
	if (mdev->dev_state == MOTOR_OPS_CRUISE) {
		mdev->dev_state = MOTOR_OPS_NORMAL;
		motors[HORIZONTAL_MOTOR].state = MOTOR_OPS_NORMAL;
		motors[VERTICAL_MOTOR].state   = MOTOR_OPS_NORMAL;
		dst->one.x = 0; cur->one.x = 0;
		dst->one.y = 0; cur->one.y = 0;
	}

	mdev->counter   = 0;
	mdev->wait_stop = 1;
	spin_unlock_irqrestore(&mdev->slock, flags);
	mutex_unlock(&mdev->dev_mutex);

	do {
		ret = wait_for_completion_interruptible_timeout(
			&mdev->stop_completion, msecs_to_jiffies(15000));
		if (ret == 0) { ret = -ETIMEDOUT; break; }
	} while (ret == -ERESTARTSYS);

	ingenic_tcu_counter_stop(mdev->tcu);
	motor_set_default(mdev);
}

static long motor_ops_goback(struct motor_device *mdev)
{
	struct motor_driver *motors = mdev->motors;
	int sx = motors[HORIZONTAL_MOTOR].max_steps >> 1;
	int sy = motors[VERTICAL_MOTOR].max_steps   >> 1;
	int cx = motors[HORIZONTAL_MOTOR].cur_steps;
	int cy = motors[VERTICAL_MOTOR].cur_steps;
	return motor_ops_move(mdev, sx - cx, sy - cy);
}

static long motor_ops_cruise(struct motor_device *mdev)
{
	unsigned long flags;
	struct motor_driver *motors = mdev->motors;

	motor_ops_goback(mdev);
	motor_power_on(mdev);

	mutex_lock(&mdev->dev_mutex);
	spin_lock_irqsave(&mdev->slock, flags);
	mdev->dev_state                = MOTOR_OPS_CRUISE;
	motors[HORIZONTAL_MOTOR].state = MOTOR_OPS_CRUISE;
	motors[VERTICAL_MOTOR].state   = MOTOR_OPS_CRUISE;
	spin_unlock_irqrestore(&mdev->slock, flags);
	mutex_unlock(&mdev->dev_mutex);

	ingenic_tcu_counter_begin(mdev->tcu);
	return 0;
}

static void motor_get_message(struct motor_device *mdev, struct motor_message *msg)
{
	struct motor_driver *motors = mdev->motors;
	msg->x           = motors[HORIZONTAL_MOTOR].cur_steps;
	msg->y           = motors[VERTICAL_MOTOR].cur_steps;
	msg->speed       = mdev->tcu_speed;
	msg->status      = (mdev->dev_state == MOTOR_OPS_STOP) ? MOTOR_IS_STOP : MOTOR_IS_RUNNING;
	msg->x_max_steps = motors[HORIZONTAL_MOTOR].max_steps;
	msg->y_max_steps = motors[VERTICAL_MOTOR].max_steps;
}

static inline int motor_ops_reset_check_params(struct motor_reset_data *rdata)
{
	if (rdata->x_max_steps == 0 || rdata->y_max_steps == 0)
		return -1;
	if (rdata->x_max_steps < rdata->x_cur_step ||
	    rdata->y_max_steps < rdata->y_cur_step)
		return -1;
	return 0;
}

static long motor_ops_reset(struct motor_device *mdev, struct motor_reset_data *rdata)
{
	unsigned long flags;
	int index;
	long ret = 0;
	int times = 0;
	struct motor_message msg;

	if (!mdev || !rdata) {
		printk("ERROR: %s got NULL pointer\n", __func__);
		return -EPERM;
	}

	if (motor_ops_reset_check_params(rdata) == 0) {
		mutex_lock(&mdev->dev_mutex);
		spin_lock_irqsave(&mdev->slock, flags);
		mdev->motors[HORIZONTAL_MOTOR].max_steps = rdata->x_max_steps;
		mdev->motors[HORIZONTAL_MOTOR].cur_steps = rdata->x_cur_step;
		mdev->motors[VERTICAL_MOTOR].max_steps   = rdata->y_max_steps;
		mdev->motors[VERTICAL_MOTOR].cur_steps   = rdata->y_cur_step;
		spin_unlock_irqrestore(&mdev->slock, flags);
		mutex_unlock(&mdev->dev_mutex);
	} else {
		motor_power_on(mdev);

		mutex_lock(&mdev->dev_mutex);
		spin_lock_irqsave(&mdev->slock, flags);

		for (index = 0; index < HAS_MOTOR_CNT; index++) {
			struct motor_driver *drv = &mdev->motors[index];
			int half = drv->max_steps > 0 ? drv->max_steps / 2 : 0;
			drv->move_dir      = MOTOR_MOVE_RIGHT_UP;
			drv->state         = MOTOR_OPS_RESET;
			drv->cur_steps     = half > 0 ? half : 0;
			drv->total_steps   = 0;
			drv->reset_max_pos = 0;
			drv->reset_min_pos = 0;
		}

		mdev->dst_move.one.x = mdev->motors[HORIZONTAL_MOTOR].max_steps;
		mdev->dst_move.one.y = mdev->motors[VERTICAL_MOTOR].max_steps;
		mdev->dst_move.times = 1;
		mdev->cur_move.one.x = 0;
		mdev->cur_move.one.y = 0;
		mdev->cur_move.times = 0;
		mdev->dev_state      = MOTOR_OPS_RESET;

		spin_unlock_irqrestore(&mdev->slock, flags);
		mutex_unlock(&mdev->dev_mutex);

		ingenic_tcu_counter_begin(mdev->tcu);

		for (index = 0; index < HAS_MOTOR_CNT; index++) {
			do {
				ret = wait_for_completion_interruptible_timeout(
					&mdev->motors[index].reset_completion,
					msecs_to_jiffies(150000));
				if (ret == 0) { ret = -ETIMEDOUT; goto exit; }
			} while (ret == -ERESTARTSYS);
		}
	}

	ret = motor_ops_move(mdev,
			     -mdev->motors[HORIZONTAL_MOTOR].cur_steps,
			     -mdev->motors[VERTICAL_MOTOR].cur_steps);
	do {
		msleep(10);
		motor_get_message(mdev, &msg);
		if (++times > 1000) { ret = -ETIMEDOUT; goto exit; }
	} while (msg.status == MOTOR_IS_RUNNING);

	ret = motor_ops_goback(mdev);
	if (ret) goto exit;
	times = 0;
	do {
		msleep(10);
		motor_get_message(mdev, &msg);
		if (++times > 1000) { ret = -ETIMEDOUT; goto exit; }
	} while (msg.status == MOTOR_IS_RUNNING);

	ret = 0;
	rdata->x_max_steps = mdev->motors[HORIZONTAL_MOTOR].max_steps;
	rdata->x_cur_step  = mdev->motors[HORIZONTAL_MOTOR].cur_steps;
	rdata->y_max_steps = mdev->motors[VERTICAL_MOTOR].max_steps;
	rdata->y_cur_step  = mdev->motors[VERTICAL_MOTOR].cur_steps;

exit:
	ingenic_tcu_counter_stop(mdev->tcu);
	msleep(10);
	motor_set_default(mdev);
	return ret;
}

static int motor_speed(struct motor_device *mdev, int speed)
{
	__asm__("ssnop");
	if (speed < MOTOR_MIN_SPEED || speed > MOTOR_MAX_SPEED) {
		dev_err(mdev->dev, "Invalid speed %d (range %d-%d)\n",
			speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
		return -1;
	}
	__asm__("ssnop");
	mdev->tcu_speed = speed;
	ingenic_tcu_set_period(mdev->tcu->cib.id, (24000000 / 64 / mdev->tcu_speed));
	return 0;
}

/* File operations */
static int motor_open(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct motor_device *mdev = container_of(dev, struct motor_device, misc_dev);
	if (mdev->flag) {
		dev_err(mdev->dev, "Motor driver busy\n");
		return -EBUSY;
	}
	mdev->flag = 1;
	return 0;
}

static int motor_release(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct motor_device *mdev = container_of(dev, struct motor_device, misc_dev);
	motor_ops_stop(mdev);
	mdev->flag = 0;
	return 0;
}

static long motor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = filp->private_data;
	struct motor_device *mdev = container_of(dev, struct motor_device, misc_dev);
	long ret = 0;

	if (!mdev->flag) {
		dev_err(mdev->dev, "Open /dev/motor first\n");
		return -EPERM;
	}

	switch (cmd) {
	case MOTOR_STOP:
		motor_ops_stop(mdev);
		break;
	case MOTOR_RESET: {
		struct motor_reset_data rdata;
		if (!arg) { ret = -EPERM; break; }
		if (copy_from_user(&rdata, (void __user *)arg, sizeof(rdata)))
			return -EFAULT;
		ret = motor_ops_reset(mdev, &rdata);
		if (!ret && copy_to_user((void __user *)arg, &rdata, sizeof(rdata)))
			return -EFAULT;
		break;
	}
	case MOTOR_MOVE: {
		struct motors_steps dst;
		if (copy_from_user(&dst, (void __user *)arg, sizeof(dst)))
			return -EFAULT;
		ret = motor_ops_move(mdev, dst.x, dst.y);
		break;
	}
	case MOTOR_GET_STATUS: {
		struct motor_message msg;
		motor_get_message(mdev, &msg);
		if (copy_to_user((void __user *)arg, &msg, sizeof(msg)))
			return -EFAULT;
		break;
	}
	case MOTOR_SPEED: {
		int speed;
		if (copy_from_user(&speed, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		motor_speed(mdev, speed);
		break;
	}
	case MOTOR_GOBACK:
		ret = motor_ops_goback(mdev);
		break;
	case MOTOR_CRUISE:
		ret = motor_ops_cruise(mdev);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct file_operations motor_fops = {
	.open           = motor_open,
	.release        = motor_release,
	.unlocked_ioctl = motor_ioctl,
};

/* Proc info */
static int motor_info_show(struct seq_file *m, void *v)
{
	struct motor_device *mdev = m->private;
	struct motor_message msg;
	int index;

	seq_printf(m, "Motor driver version: %s\n", JZ_MOTOR_DRIVER_VERSION);
	seq_printf(m, "Driver state: %s\n", mdev->flag ? "opened" : "closed");
	seq_printf(m, "Speed range: %d - %d\n", MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
	motor_get_message(mdev, &msg);
	seq_printf(m, "Status: %s\n", msg.status ? "running" : "stop");
	seq_printf(m, "Position: (%d, %d)\n", msg.x, msg.y);
	seq_printf(m, "Speed: %d\n", msg.speed);
	for (index = 0; index < HAS_MOTOR_CNT; index++) {
		seq_printf(m, "## %s ##\n", mdev->motors[index].pdata->name);
		seq_printf(m, "  max_steps: %d  cur_steps: %d\n",
			   mdev->motors[index].max_steps,
			   mdev->motors[index].cur_steps);
		seq_printf(m, "  GPIOs: ST1=%d ST2=%d ST3=%d ST4=%d\n",
			   mdev->motors[index].pdata->motor_st1_gpio,
			   mdev->motors[index].pdata->motor_st2_gpio,
			   mdev->motors[index].pdata->motor_st3_gpio,
			   mdev->motors[index].pdata->motor_st4_gpio);
	}
	return 0;
}

static int motor_info_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, motor_info_show, PDE_DATA(inode), 1024);
}

static const struct file_operations motor_info_fops = {
	.read    = seq_read,
	.open    = motor_info_open,
	.llseek  = seq_lseek,
	.release = single_release,
};

/* Platform driver */
static int motor_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct motor_device *mdev;
	struct motor_driver *motor;
	struct proc_dir_entry *proc;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev) {
		dev_err(&pdev->dev, "Failed to allocate motor_device\n");
		return -ENOMEM;
	}

	mdev->cell = mfd_get_cell(pdev);
	if (!mdev->cell) {
		dev_err(&pdev->dev, "Failed to get MFD cell\n");
		return -ENOENT;
	}

	mdev->dev = &pdev->dev;
	mdev->tcu = (struct ingenic_tcu_chn *)mdev->cell->platform_data;

	mdev->tcu->irq_type  = FULL_IRQ_MODE;
	mdev->tcu->clk_src   = TCU_CLKSRC_EXT;
	mdev->tcu->is_pwm    = 0;
	mdev->tcu->cib.func  = TRACKBALL_FUNC;
	mdev->tcu->clk_div   = TCU_PRESCALE_64;
	mdev->tcu_speed      = MOTOR_MAX_SPEED;

	ingenic_tcu_config(mdev->tcu);
	ingenic_tcu_set_period(mdev->tcu->cib.id, (24000000 / 64 / mdev->tcu_speed));

	mutex_init(&mdev->dev_mutex);
	spin_lock_init(&mdev->slock);
	init_completion(&mdev->stop_completion);
	mdev->wait_stop = 0;

	platform_set_drvdata(pdev, mdev);

	/* Apply module parameters */
	motors_pdata[HORIZONTAL_MOTOR].motor_st1_gpio    = hst1;
	motors_pdata[HORIZONTAL_MOTOR].motor_st2_gpio    = hst2;
	motors_pdata[HORIZONTAL_MOTOR].motor_st3_gpio    = hst3;
	motors_pdata[HORIZONTAL_MOTOR].motor_st4_gpio    = hst4;
	motors_pdata[HORIZONTAL_MOTOR].motor_min_gpio    = hmin;
	motors_pdata[HORIZONTAL_MOTOR].motor_max_gpio    = hmax;
	motors_pdata[HORIZONTAL_MOTOR].motor_switch_gpio = motor_switch_gpio;

	motors_pdata[VERTICAL_MOTOR].motor_st1_gpio      = vst1;
	motors_pdata[VERTICAL_MOTOR].motor_st2_gpio      = vst2;
	motors_pdata[VERTICAL_MOTOR].motor_st3_gpio      = vst3;
	motors_pdata[VERTICAL_MOTOR].motor_st4_gpio      = vst4;
	motors_pdata[VERTICAL_MOTOR].motor_min_gpio      = vmin;
	motors_pdata[VERTICAL_MOTOR].motor_max_gpio      = vmax;

	if (invert_gpio_dir != 0) invert_gpio_dir = 1;

	if (motor_switch_gpio != -1)
		gpio_request(motor_switch_gpio, "motor_switch_gpio");

	for (i = 0; i < HAS_MOTOR_CNT; i++) {
		motor = &mdev->motors[i];
		motor->pdata    = &motors_pdata[i];
		motor->move_dir = MOTOR_MOVE_STOP;
		init_completion(&motor->reset_completion);

		dev_info(&pdev->dev, "'%s' GPIOs: ST1=%d ST2=%d ST3=%d ST4=%d\n",
			 motor->pdata->name,
			 motor->pdata->motor_st1_gpio,
			 motor->pdata->motor_st2_gpio,
			 motor->pdata->motor_st3_gpio,
			 motor->pdata->motor_st4_gpio);

		if (motor->pdata->motor_st1_gpio != -1)
			gpio_request(motor->pdata->motor_st1_gpio, "motor_st1_gpio");
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_request(motor->pdata->motor_st2_gpio, "motor_st2_gpio");
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_request(motor->pdata->motor_st3_gpio, "motor_st3_gpio");
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_request(motor->pdata->motor_st4_gpio, "motor_st4_gpio");
	}

	/* Seed max_steps with buffer for reset sweep */
	mdev->motors[HORIZONTAL_MOTOR].max_steps = hmaxstep + 100;
	mdev->motors[VERTICAL_MOTOR].max_steps   = vmaxstep + 30;

	/* Seed cur_steps at mid-travel */
	mdev->motors[HORIZONTAL_MOTOR].cur_steps =
		mdev->motors[HORIZONTAL_MOTOR].max_steps / 2;
	mdev->motors[VERTICAL_MOTOR].cur_steps =
		mdev->motors[VERTICAL_MOTOR].max_steps / 2;

	jzgpio_set_func(GPIO_PORT_C, GPIO_PULL_UP, 1 << 13);
	jzgpio_set_func(GPIO_PORT_C, GPIO_PULL_UP, 1 << 14);
	jzgpio_set_func(GPIO_PORT_C, GPIO_PULL_UP, 1 << 18);

	ingenic_tcu_channel_to_virq(mdev->tcu);
	mdev->run_step_irq = mdev->tcu->virq[0];
	if (mdev->run_step_irq < 0) {
		ret = mdev->run_step_irq;
		dev_err(&pdev->dev, "Failed to get TCU IRQ: %d\n", ret);
		goto error_get_irq;
	}

	ret = request_irq(mdev->run_step_irq, jz_timer_interrupt, 0,
			  "jz_timer_interrupt", mdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		goto error_request_irq;
	}

	mdev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	mdev->misc_dev.name  = "motor";
	mdev->misc_dev.fops  = &motor_fops;
	ret = misc_register(&mdev->misc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "misc_register failed\n");
		goto error_misc_register;
	}

	proc = jz_proc_mkdir("motor");
	mdev->proc = proc;
	if (proc)
		proc_create_data("motor_info", S_IRUGO, proc,
				 &motor_info_fops, mdev);

	motor_set_default(mdev);
	mdev->flag = 0;

	ingenic_tcu_counter_begin(mdev->tcu);

	printk("motor: probe ok — pan %d steps (GPIOs %d/%d/%d/%d), "
	       "tilt %d steps (GPIOs %d/%d/%d/%d)\n",
	       hmaxstep, hst1, hst2, hst3, hst4,
	       vmaxstep, vst1, vst2, vst3, vst4);
	return 0;

error_misc_register:
	free_irq(mdev->run_step_irq, mdev);
error_request_irq:
error_get_irq:
	for (i = 0; i < HAS_MOTOR_CNT; i++) {
		motor = &mdev->motors[i];
		if (!motor->pdata) continue;
		if (motor->pdata->motor_st1_gpio != -1)
			gpio_free(motor->pdata->motor_st1_gpio);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_free(motor->pdata->motor_st2_gpio);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_free(motor->pdata->motor_st3_gpio);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_free(motor->pdata->motor_st4_gpio);
	}
	return ret;
}

static int motor_remove(struct platform_device *pdev)
{
	int i;
	struct motor_device *mdev = platform_get_drvdata(pdev);
	struct motor_driver *motor;

	ingenic_tcu_counter_stop(mdev->tcu);
	mutex_destroy(&mdev->dev_mutex);
	free_irq(mdev->run_step_irq, mdev);

	for (i = 0; i < HAS_MOTOR_CNT; i++) {
		motor = &mdev->motors[i];
		if (!motor->pdata) continue;
		if (motor->pdata->motor_switch_gpio != -1)
			gpio_free(motor->pdata->motor_switch_gpio);
		if (motor->pdata->motor_st1_gpio != -1)
			gpio_free(motor->pdata->motor_st1_gpio);
		if (motor->pdata->motor_st2_gpio != -1)
			gpio_free(motor->pdata->motor_st2_gpio);
		if (motor->pdata->motor_st3_gpio != -1)
			gpio_free(motor->pdata->motor_st3_gpio);
		if (motor->pdata->motor_st4_gpio != -1)
			gpio_free(motor->pdata->motor_st4_gpio);
	}

	if (mdev->proc)
		proc_remove(mdev->proc);

	misc_deregister(&mdev->misc_dev);
	return 0;
}

/* Module init/exit */
static struct of_device_id motor_match_dynamic[2];
static char motor_match_str[32];
static char motor_driver_name[32];

static struct platform_driver motor_driver = {
	.probe  = motor_probe,
	.remove = motor_remove,
	.driver = {
		.name           = motor_driver_name,
		.of_match_table = motor_match_dynamic,
		.owner          = THIS_MODULE,
	},
};

static int __init motor_init(void)
{
	snprintf(motor_driver_name, sizeof(motor_driver_name),
		 "ingenic,tcu_chn%d", motor_bind_channel);
	snprintf(motor_match_str, sizeof(motor_match_str),
		 "ingenic,tcu_chn%d", motor_bind_channel);
	memset(motor_match_dynamic, 0, sizeof(motor_match_dynamic));
	motor_match_dynamic[0].compatible = motor_match_str;
	motor_match_dynamic[1].compatible = NULL;
	printk("motor: registering driver for '%s'\n", motor_driver_name);
	return platform_driver_register(&motor_driver);
}

static void __exit motor_exit(void)
{
	platform_driver_unregister(&motor_driver);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_LICENSE("GPL");
