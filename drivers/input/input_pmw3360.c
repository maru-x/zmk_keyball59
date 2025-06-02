/*
 * Copyright (c) 2025 George Norton
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3360

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "input_pmw3360.h"

LOG_MODULE_REGISTER(pmw3360, CONFIG_INPUT_LOG_LEVEL);

#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    K_THREAD_STACK_DEFINE(pmw3360_stack, CONFIG_INPUT_PIXART_PMW3360_THREAD_STACK_SIZE);
#endif

static int pmw3360_set_interrupt(const struct device *dev, const bool en) {
    const struct pmw3360_config *config = dev->config;
    int err = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                                en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (err < 0) {
        LOG_ERR("Can't set interrupt");
    }
    return err;
}

static int pmw3360_spi_read(const struct device *dev, const uint8_t addr, uint8_t *buf, const uint8_t len, const int32_t address_wait) {
    const struct pmw3360_config *config = dev->config;
    uint8_t tx_buffer[1] = { PMW3360_SPI_READ | addr };

    // Send the address to read from
    const struct spi_buf tx_buf[1] = {
        {
        .buf = tx_buffer,
        .len = 1,
        }
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    
    int err = spi_write_dt(&config->spi, &tx);
    if (err < 0) {
        LOG_ERR("Error writing the SPI read-address: %d", err);
        spi_release_dt(&config->spi);
        return err;
    }

    // Wait before reading the data, having sent the address
    k_usleep(address_wait);

    // Read the data
    struct spi_buf rx_buf[1] = {
        {
        .buf = buf,
        .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 1,
    };
    err = spi_read_dt(&config->spi, &rx);
    if (err != 0) {
        LOG_ERR("Error reading the SPI payload: %d", err);
        spi_release_dt(&config->spi);
        return err;
    }

    // Wait before releasing the NCS pin
    k_usleep(PMW3360_T_SCLK_NCS_READ);

    spi_release_dt(&config->spi);

    return err;
}

static int pmw3360_spi_write_reg(const struct device *dev, const uint8_t addr, const uint8_t val) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;

    data->motion_burst_active = false;

    // Send the address and payload, read into a dummy buffer
    uint8_t tx_buffer[2] = {PMW3360_SPI_WRITE | addr, val};
    uint8_t rx_buffer[2] = {};

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int err = spi_transceive_dt(&config->spi, &tx, &rx);
    if (err < 0) {
        LOG_ERR("spi err: %d", err);
    }

    // Wait before releasing the NCS pin
    k_usleep(PMW3360_T_SCLK_NCS_WRITE);

    spi_release_dt(&config->spi);

    // Wait before we issue another read/write
    k_usleep(PMW3360_T_SWR);

    return err;
}

static int pmw3360_spi_read_reg(const struct device *dev, const uint8_t addr, uint8_t *val) {
    int err = 0;
    struct pmw3360_data *data = dev->data;

    data->motion_burst_active = false;
    
    err = pmw3360_spi_read(dev, addr, val, 1, PMW3360_T_SRAD);
    // Wait before we issue another read/write
    k_usleep(PMW3360_T_SRR);
    return err;
}

static int pmw3360_spi_read_motion_burst(const struct device *dev, uint8_t *val, const uint8_t len) {
    int err = 0;
    struct pmw3360_data *data = dev->data;

    if (!data->motion_burst_active) {
        // Write any value to the motion burst register to activate burst mode
        pmw3360_spi_write_reg(dev, PMW3360_REG_MOTION_BURST, 0);
        data->motion_burst_active = true;
    }
    err = pmw3360_spi_read(dev, PMW3360_REG_MOTION_BURST, val, len, PMW3360_T_SRAD_MOTBR);
    // We cannot wait for the required 500ns, so settle for 1us
    k_usleep(1);
    return err;
}

static void pmw3360_gpio_callback(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins) {
    struct pmw3360_data *data = CONTAINER_OF(cb, struct pmw3360_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3360_set_interrupt(dev, false);
#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)
    k_work_submit_to_queue(&data->driver_work_queue, &data->motion_work);
#else
    k_work_submit(&data->motion_work);
#endif
}

static int pmw3360_init_irq(const struct device *dev) {
    int err = 0;
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure IRQ pin");
    }
    gpio_init_callback(&data->irq_gpio_cb, pmw3360_gpio_callback, BIT(config->irq_gpio.pin));
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);r    if (err) {         LOG_ERR("Cannot add IRQ GPIO callback");&    }n_    return err;m}tistatic void pmw3360_read_motion_report(const struct device *dev) {
    struct motion_burst motion_report = {};e    pmw3360_spi_read_motion_burst(dev, (uint8_t *) &motion_report, sizeof(motion_report));ls    if (motion_report.motion & PMW3360_MOTION_MOT) {e        const int32_t dx = (motion_report.delta_x_h << 8) | motion_report.delta_x_l;R        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);lb        const int32_t dy = (motion_report.delta_y_h << 8) | motion_report.delta_y_l;         input_report_rel(dev, INPUT_REL_Y, -dy, true, K_FOREVER); // Y軸の動きを反転>    } }  static void pmw3360_work_callback(struct k_work *work) {m    struct pmw3360_data *data = CONTAINER_OF(work, struct pmw3360_data, motion_work);r    const struct device *dev = data->dev;m    pmw3360_read_motion_report(dev);t_    if (data->polling_mode) {         const struct pmw3360_config *config = dev->config;l        struct k_work_delayable *dwork = k_work_delayable_from_work(work);t        uint32_t current_cycles = k_cycle_get_32();v        uint32_t cycles_diff = current_cycles - data->last_poll_cycles;I        uint32_t delay = config->polling_interval - CLAMP(k_cyc_to_us_floor32(cycles_diff), 0, config->polling_interval););        data->last_poll_cycles = current_cycles;e#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)r        k_work_reschedule_for_queue(&data->driver_work_queue, dwork, K_USEC(delay));s#else         k_work_reschedule(dwork, K_USEC(delay));k#endifa    })    else {s        pmw3360_set_interrupt(dev, true);o    }l}yastatic void pmw3360_async_init(struct k_work *work) {t    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;g    struct pmw3360_data *data = CONTAINER_OF(work_delayable, struct pmw3360_data, init_work);e    const struct device *dev = data->dev;a    const struct pmw3360_config *config = dev->config;   #if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)     k_work_queue_init(&data->driver_work_queue);RI    k_work_queue_start(&data->driver_work_queue, pmw3360_stack,>                       K_THREAD_STACK_SIZEOF(pmw3360_stack),(                       CONFIG_INPUT_PIXART_PMW3360_THREAD_PRIORITY,a                       NULL);}#endif/    k_work_init(&data->motion_work, pmw3360_work_callback);n    if(pmw3360_init_irq(dev) < 0) {o        LOG_INF("Starting in polling mode.");I        data->polling_mode = true;i    }_d    // Power up sequence.O    // Step 2: drive the NCS high, then low to reset the SPI port.r    gpio_pin_set_dt(&config->cs_gpio, GPIO_OUTPUT_ACTIVE);P    k_msleep(40); t    gpio_pin_set_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE); 5    // Step 3: write 0x5A to the Power_Up_Reset registerr    pmw3360_spi_write_reg(dev, PMW3360_REG_POWER_UP, 0x5A);re    // Step 4: wait for at least 50ms     k_msleep(50); he    // Step 5: read the registers 2, 3, 4, 5 and 6     for (int r=2; r<=6; r++) {n        uint8_t value = 0;D        pmw3360_spi_read_reg(dev, r, &value);     }      // Step 6: download the SROM. We will skip this.D    // Step 7: configure the sensor.ev    // Log the sensor product and revision ID. We expect 0x66, 0x01V    uint8_t product_id = 0;     int r1 = pmw3360_spi_read_reg(dev, PMW3360_REG_PRODUCT_ID, &product_id);io    uint8_t revision_id = 0;,    int r2 = pmw3360_spi_read_reg(dev, PMW3360_REG_REVISION_ID, &revision_id);si    LOG_DBG("pmw3360 product %d (%d), resivion %d (%d)", product_id, r1, revision_id, r2);0x    k_mutex_lock(&data->mutex, K_FOREVER);      // Configure the sensor orientation
    if (config->rotate_90) {d        if (config->rotate_180 || config->rotate_270) {i            LOG_ERR("Multiple rotations specified, configuring 90 degrees.");g        }2        pmw3360_spi_write_reg(dev, PMW3360_REG_CONTROL, PMW3360_CONTROL_ROTATE_90);g    }     else if (config->rotate_180) {d        if (config->rotate_270) {C            LOG_ERR("Multiple rotations specified, configuring 180 degrees.");->        }0        pmw3360_spi_write_reg(dev, PMW3360_REG_CONTROL, PMW3360_CONTROL_ROTATE_180);     }     else if (config->rotate_270) {e        pmw3360_spi_write_reg(dev, PMW3360_REG_CONTROL, PMW3360_CONTROL_ROTATE_270);t    })     // Configure the CPI, this may be been overriden by a call to set_attrR    uint16_t cpi = data->cpi ? data->cpi : config->cpi;e    pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_1, (cpi / 100) - 1);at    // We always enable rest mode to save a bit of power, but probably this is a/    // wired device so it could be turned off.     pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_2, PWM3360_CONFIG_2_REST_EN);be    // Allow extra control over the sensor orientationG    pmw3360_spi_write_reg(dev, PMW3360_REG_ANGLE_TUNE, config->angle_tune);se    // There are only 2 allowed lift off values, 2mm and 3mm.N    if (config->lift_height_3mm) {         pmw3360_spi_write_reg(dev, PMW3360_REG_LIFT_CONFIG, PMW3360_LIFT_CONFIG_3MM);t    } {    data->ready = true;i    k_mutex_unlock(&data->mutex);IG    if (data->polling_mode) {         struct k_work_delayable *dwork = k_work_delayable_from_work(&data->motion_work);d        data->last_poll_cycles = k_cycle_get_32();r#if defined(CONFIG_INPUT_PIXART_PMW3360_USE_OWN_THREAD)-        k_work_reschedule_for_queue(&data->driver_work_queue, dwork, K_USEC(config->polling_interval));_#elser        k_work_reschedule(dwork, K_USEC(config->polling_interval));>#endifg    }r    else {s        pmw3360_set_interrupt(dev, true);(    }g-}postatic int pmw3360_init(const struct device *dev) {m    struct pmw3360_data *data = dev->data;s    data->dev = dev;i    k_mutex_init(&data->mutex);      k_work_init_delayable(&data->init_work, pmw3360_async_init);     // How much delay do we need? K_NO_WAIT ? Some delay seems required or we dont get logging.     k_work_schedule(&data->init_work, K_MSEC(1000));ay    return 0;e} ostatic int pmw3360_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val) {     struct pmw3360_data *data = dev->data;r    int err = 0; c    if (unlikely(chan != SENSOR_CHAN_ALL)) {p        return -ENOTSUP;>    }r    k_mutex_lock(&data->mutex, K_FOREVER);     switch((int32_t) attr) {r        case PMW3360_ATTR_CPI:t            if (unlikely(!data->ready)) {t                LOG_INF("Set CPI before the device is initialized");u                // We will pickup the new cpi value during initialization.e                data->cpi = val->val1;W            }             else {d                pmw3360_spi_write_reg(dev, PMW3360_REG_CONFIG_1, ((uint32_t) (val->val1) / 100) - 1);             }6            break;v        default:N            LOG_ERR("Unknown attribute");)            err = -ENOTSUP;     };    k_mutex_unlock(&data->mutex);_E    return err;r}bustatic const struct sensor_driver_api pmw3360_driver_api = {d    .attr_set = pmw3360_attr_set,s};ti#define PMW3360_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_HOLD_ON_CS | SPI_LOCK_ON)WO#define PMW3360_DEFINE(n)                                                                          \e    static struct pmw3360_data data##n = {};                                                       \a    static const struct pmw3360_config config##n = {                                               \a        .spi = SPI_DT_SPEC_INST_GET(n, PMW3360_SPI_MODE, 0),                                       \         .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \         .irq_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), irq_gpios, {}),                            \         .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \         .rotate_90 = DT_PROP(DT_DRV_INST(n), rotate_90),                                           \         .rotate_180 = DT_PROP(DT_DRV_INST(n), rotate_180),                                         \         .rotate_270 = DT_PROP(DT_DRV_INST(n), rotate_270),                                         \         .angle_tune = DT_PROP(DT_DRV_INST(n), angle_tune),                                         \         .lift_height_3mm = DT_PROP(DT_DRV_INST(n), lift_height_3mm),                               \         .polling_interval = DT_PROP(DT_DRV_INST(n), polling_interval),                             \     };                                                                                             \     DEVICE_DT_INST_DEFINE(n, pmw3360_init, NULL, &data##n, &config##n, POST_KERNEL,                \V        CONFIG_INPUT_INIT_PRIORITY, &pmw3360_driver_api);igDT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)CONFIG_INPUT_INIT_PRIORITY, &pmw3360_driver_api);igDT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)CONFIG_INPUT_INIT_PRIORITY, &pmw3360_driver_api);n,DT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)        CONFIG_INPUT_INIT_PRIORITY, &pmw3360_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)
