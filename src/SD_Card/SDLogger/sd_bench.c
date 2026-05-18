/**
 * Standalone SD card write benchmark.
 *
 * Writes blocks directly to a file on the SD card (bypassing the ring buffer
 * and zbus pipeline) and reports per-write latency statistics.  This lets us
 * measure raw SD write performance independently of the sensor stack.
 *
 * Usage:  sd_bench <duration_s> [block_size] [interval_us]
 *
 *   duration_s  – how long to run (seconds)
 *   block_size  – bytes per write (default 512)
 *   interval_us – delay between writes in µs (default 1000 = 1 ms)
 *
 * Example: sd_bench 10 4096 2000
 *          → 10 s of 4 KB writes every 2 ms ≈ 2 MB/s
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <ff.h>
#include <string.h>

LOG_MODULE_REGISTER(sd_bench, LOG_LEVEL_INF);

/* Histogram buckets (µs): <500, <1000, <2000, <5000, <10000, <50000, >=50000 */
#define HIST_BUCKETS 7
static const uint32_t hist_limits[HIST_BUCKETS] = {
    500, 1000, 2000, 5000, 10000, 50000, UINT32_MAX
};
static const char *hist_labels[HIST_BUCKETS] = {
    "<0.5ms", "<1ms", "<2ms", "<5ms", "<10ms", "<50ms", ">=50ms"
};

static int cmd_sd_bench(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: sd_bench <duration_s> [block_size] [interval_us]");
        return -EINVAL;
    }

    int duration_s   = atoi(argv[1]);
    int block_size   = (argc > 2) ? atoi(argv[2]) : 512;
    int interval_us  = (argc > 3) ? atoi(argv[3]) : 1000;

    if (duration_s <= 0 || block_size <= 0 || block_size > 16384) {
        shell_error(sh, "Bad params (block_size max 16384)");
        return -EINVAL;
    }

    /* Fill a write buffer with a repeating pattern */
    static uint8_t wbuf[16384];
    memset(wbuf, 0xAA, sizeof(wbuf));

    /* Mount SD card only if nothing else has already mounted "/SD:".
     * Zephyr's fs_mount logs "file system already mounted!!" at <err>
     * whenever a mount struct is re-registered, so we query the live
     * mount table rather than tracking our own flag. */
    static FATFS fat_fs;
    static struct fs_mount_t mnt = {
        .type = FS_FATFS,
        .fs_data = &fat_fs,
        .mnt_point = "/SD:",
    };
    bool already_mounted = false;
    int idx = 0;
    const char *mname;
    while (fs_readmount(&idx, &mname) == 0) {
        if (strcmp(mname, mnt.mnt_point) == 0) {
            already_mounted = true;
            break;
        }
    }
    bool we_mounted = false;
    if (!already_mounted) {
        int mret = fs_mount(&mnt);
        if (mret < 0) {
            shell_error(sh, "SD mount failed: %d", mret);
            return mret;
        }
        we_mounted = true;
    }

    /* Open a test file */
    struct fs_file_t fp;
    fs_file_t_init(&fp);

    const char *path = "/SD:/sd_bench.bin";
    int ret = fs_open(&fp, path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (ret < 0) {
        shell_error(sh, "fs_open failed: %d", ret);
        return ret;
    }

    shell_print(sh, "SD bench: %d s, %d B/write, %d us interval",
                duration_s, block_size, interval_us);

    /* Stats */
    uint32_t hist[HIST_BUCKETS] = {0};
    uint32_t count = 0;
    uint64_t total_us = 0;
    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;
    uint64_t total_bytes = 0;

    /* Collect the 16 largest write times for reporting */
    uint32_t top[16] = {0};

    uint64_t end_time = k_uptime_get() + (uint64_t)duration_s * 1000;

    while (k_uptime_get() < end_time) {
        uint64_t t0 = k_cycle_get_32();

        ret = fs_write(&fp, wbuf, block_size);

        uint64_t t1 = k_cycle_get_32();

        if (ret < 0) {
            shell_error(sh, "fs_write failed: %d", ret);
            break;
        }

        uint32_t elapsed_us = k_cyc_to_us_floor32(t1 - t0);

        count++;
        total_us += elapsed_us;
        total_bytes += ret;
        if (elapsed_us < min_us) min_us = elapsed_us;
        if (elapsed_us > max_us) max_us = elapsed_us;

        /* Histogram */
        for (int i = 0; i < HIST_BUCKETS; i++) {
            if (elapsed_us < hist_limits[i]) {
                hist[i]++;
                break;
            }
        }

        /* Track top-N worst writes */
        for (int i = 0; i < 16; i++) {
            if (elapsed_us > top[i]) {
                /* Shift down */
                for (int j = 15; j > i; j--) top[j] = top[j-1];
                top[i] = elapsed_us;
                break;
            }
        }

        if (interval_us > 0) {
            k_usleep(interval_us);
        }
    }

    fs_sync(&fp);
    fs_close(&fp);
    /* Clean up test file */
    fs_unlink(path);

    /* Only unmount if we were the ones who mounted — leave any pre-existing
     * owner (e.g. sd_card module) undisturbed. */
    if (we_mounted) {
        int uret = fs_unmount(&mnt);
        if (uret < 0) {
            shell_warn(sh, "SD unmount returned %d", uret);
        }
    }

    if (count == 0) {
        shell_print(sh, "No writes completed");
        return 0;
    }

    uint32_t mean_us = (uint32_t)(total_us / count);

    shell_print(sh, "\n=== SD Bench Results ===");
    shell_print(sh, "  Writes: %u (%llu bytes, %.1f KB/s)",
                count, total_bytes,
                (double)total_bytes / duration_s / 1024.0);
    shell_print(sh, "  Latency (us): min=%u  mean=%u  max=%u",
                min_us, mean_us, max_us);

    shell_print(sh, "\n  Histogram:");
    for (int i = 0; i < HIST_BUCKETS; i++) {
        if (hist[i] > 0) {
            shell_print(sh, "    %7s: %u (%.1f%%)",
                        hist_labels[i], hist[i],
                        (double)hist[i] / count * 100.0);
        }
    }

    shell_print(sh, "\n  Top-16 worst writes (us):");
    char line[128];
    int pos = 0;
    for (int i = 0; i < 16 && top[i] > 0; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%u ", top[i] / 1000);
    }
    shell_print(sh, "    %s (ms)", line);

    return 0;
}

SHELL_CMD_ARG_REGISTER(sd_bench, NULL,
    "SD write benchmark: sd_bench <duration_s> [block_size] [interval_us]",
    cmd_sd_bench, 2, 2);
