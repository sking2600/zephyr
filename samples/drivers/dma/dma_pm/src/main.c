#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <stdio.h>

#define LED0_NODE DT_ALIAS(led0)

int main(void)
{
	const struct device *dma = DEVICE_DT_GET(DT_NODELABEL(dma1));
	const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
	int ret;

	printf("WCH DMA PM Verification Sample (Looping)\n");

	if (!device_is_ready(dma)) {
		printf("DMA device not ready\n");
		return -1;
	}

	if (!gpio_is_ready_dt(&led)) {
		printf("LED GPIO not ready\n");
		return -1;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	while (1) {
		printf("Suspending DMA...\n");
		ret = pm_device_action_run(dma, PM_DEVICE_ACTION_SUSPEND);
		printf("Suspend ret: %d\n", ret);
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_MSEC(500));

		printf("Resuming DMA...\n");
		ret = pm_device_action_run(dma, PM_DEVICE_ACTION_RESUME);
		printf("Resume ret: %d\n", ret);
		gpio_pin_set_dt(&led, 0);
		k_sleep(K_MSEC(500));
	}

	return 0;
}
