#ifndef _SD_CARD_MANAGER_H_
#define _SD_CARD_MANAGER_H_

#include <stddef.h>
#include <string>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/fs/fs.h>
#include <ff.h>

/**
 * @brief Class to manage the sd card.
 * 
 * @details This class provides functions to mount the sd card, open directories, list files, create directories,
 *      open files, write to files, flush the cache, and remove files or directories.
 * @author Dennis Moschina
 */
class SDCardManager {
public:
    SDCardManager();
    ~SDCardManager();

    void init();

    /**
     * @brief Mount the sd card.
     * 
     * @details Even after an error, it is possible that the sd card is mounted.
     *      Check the return value of is_mounted() to verify if the sd card is mounted.
     * 
     * @return < 0 on error, 0 on success
     */
    int mount();
    /**
     * @brief Unmount the sd card.
     * 
     * @return < 0 on error, 0 on success
     */
    int unmount();
    /**
     * @brief Open a directory.
     * 
     * @param path The path to the directory.
     *          An empty string opens the root directory,
     *          a path starting with '/' opens the directory from the root.
     *          All other open the path in the current directory.
     * @return 0 on success, < 0 on error
     */
    int cd(std::string path);
    /**
     * @brief List the files in the current directory.
     * 
     * @param buf the buffer to store the file names
     * @param buf_size the size of the buffer
     * @return 0 on success, < 0 on error
     */
    int ls(char *buf, size_t *buf_size);
    /**
     * @brief Create a directory.
     * 
     * @param path the path to the directory to create
     * @return 0 on success, < 0 on error
     */
    int mkdir(std::string path);

    /**
     * @brief Open a file.
     * 
     * @param path the path to the file
     * @param write open the file for writing
     * @param append append to the file
     * @param create create the file if it does not exist
     * @return 0 on success, < 0 on error
     */
    int open_file(std::string path, bool write, bool append, bool create);
    /**
     * @brief Close the currently opened file.
     * 
     * @return 0 on success, < 0 on error
     */
    int close_file();

    /**
     * @brief Write to the currently opened file.
     * 
     * @param buf the buffer to write
     * @param buf_size the size of the buffer
     * @param sync flush the cache after write
     * @return the number of bytes written, < 0 on error
     */
    ssize_t write(char *buf, size_t *buf_size, bool sync = false);
    /**
     * @brief Write to a file.
     * 
     * @param path the path to the file
     * @param buf the buffer to write
     * @param buf_size the size of the buffer
     * @param append append to the file
     * @return the number of bytes written, < 0 on error
     */
    ssize_t write(std::string path, char *buf, size_t *buf_size, bool append = false);
    /**
     * @brief Flushes the cache of the opened file.
     * 
     * @return 0 on success, < 0 on error
     */
    int sync();
    /**
     * @brief Remove a file or directory.
     * 
     * @param path the path to the file or directory
     * @return 0 on success, < 0 on error
     */
    int rm(std::string path);

    /**
     * @brief Read the contents of the currently opened file into a buffer.
     * 
     * @details In order to read the full file,
     *          just call this function until the number of bytes read is 0.
     * 
     * @param buf the buffer to store the file contents in
     * @param buf_size the length of the buffer.
     *                  The number of bytes read is stored in this variable.
     * @return 0 on success, -1 if no file is open, < 0 on error
     */
    int read(char *buf, size_t *buf_size);


    /**
     * @brief Write to the currently opened file.
     * 
     * @param path the path to the file
     * @param buf the buffer to write
     * @param buf_size the size of the buffer
     * @param append append to the file
     * @param create create the file if it does not exist
     * @return the number of bytes written, < 0 on error
     */
    int open_write_close(std::string path, char *buf, size_t *buf_size, bool append = false, bool create = false);

    /**
     * @brief Open the file provided in path and read the contents of the currently opened file into a buffer.
     * 
     * @details In order to read the full file,
     *          just call this function until the number of bytes read is 0.
     * 
     * @param buf the buffer to store the file contents in
     * @param buf_size the length of the buffer.
     *                  The number of bytes read is stored in this variable.
     * @return 0 on success, -1 if no file is open, < 0 on error
     */
    int open_read_close(std::string path, char *buf, size_t *buf_size);

    /**
     * @brief Check if the sd card is mounted.
     * 
     * @return true if the sd card is mounted, false otherwise
     */
    bool is_mounted() { return this->mounted; }

private:
    std::string path;
    bool mounted = false;
    struct fs_dir_t dirp;

    gpio_callback sd_state_cb;
    const struct gpio_dt_spec sd_state_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(sd_state), gpios);

    static void sd_card_state_change_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    FATFS fat_fs;
    struct fs_mount_t mnt_pt = {
        .type = FS_FATFS,
        .fs_data = &fat_fs,
    };

    struct tracked_fs_file_t {
        struct fs_file_t filep;
        bool is_open;
    } tracked_file = {
        .is_open = false,
    };

    static k_work_delayable unmount_work;

    static void unmount_work_handler(struct k_work *work);

    int aquire_ls();
    int release_ls();

    bool ls_aquired = false;

    bool sd_inserted();

};

extern SDCardManager sdcard_manager;

#endif