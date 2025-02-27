#ifndef _CSV_LOGGER_H_
#define _CSV_LOGGER_H_

#include <string>

#include "SD_Card_Manager.hpp"

class CSVLogger {
public:
    /**
     * @brief Construct a new CSVLogger object
     * 
     * @param path the path where the file should be saved
     * @param headers the array of headers of the csv file
     * @param header_count the number of columns
     * @param separator the separator between the columns
     */
    CSVLogger(std::string path, SDCardManager *sd_card, const char **headers, size_t header_count, char separator = ',');
    ~CSVLogger();

    int begin();
    int end();

    int sync();

    int writeLine(const char **data, bool sync = false);
    int writeLine(const double *data, bool sync = false);
    int writeLine(const int *data, bool sync = false);

private:
    std::string file_path;
    const char **headers;
    size_t header_count;
    char separator;

    SDCardManager *sd_card;

    int write(std::string line, bool sync = false);
};

#endif