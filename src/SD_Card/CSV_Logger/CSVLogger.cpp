#include "CSVLogger.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(CSVLogger, LOG_LEVEL_DBG);

template<typename T>
std::string joinLine(const T *data, size_t header_count, char separator) {
    std::string line = "";
    for (size_t i = 0; i < header_count; i++) {
        line += std::to_string(data[i]);
        if (i < header_count - 1) {
            line += separator;
        }
    }
    return line;
}

std::string joinLine(const char **data, size_t header_count, char separator) {
    std::string line = "";
    for (size_t i = 0; i < header_count; i++) {
        line += data[i];
        if (i < header_count - 1) {
            line += separator;
        }
    }
    return line;
}


CSVLogger::CSVLogger(std::string path, SDCardManager *sd_card , const char **headers, size_t header_count, char separator) {
    this->file_path = path;
    this->headers = headers;
    this->header_count = header_count;
    this->separator = separator;
    this->sd_card = sd_card;
}

CSVLogger::~CSVLogger() {
    this->end();
}

int CSVLogger::begin() {
    // TODO: check if file already exists
    int ret = sd_card->open_file(this->file_path, true, true, true);
    if (ret != 0) {
        LOG_ERR("Failed to open file: %d", ret);
    } else {
        LOG_DBG("Opened file %s", this->file_path.c_str());

        std::string header_str = joinLine(this->headers, this->header_count, this->separator);
        header_str += "\n";
        LOG_DBG("Header: %s", header_str.c_str());
        size_t header_size = header_str.size();
        this->sd_card->write((char *)header_str.c_str(), &header_size);
    }
    return ret;
}

int CSVLogger::end() {
    int ret = sd_card->close_file();
    if (ret != 0) {
        LOG_ERR("Failed to close file: %d", ret);
    } else {
        LOG_DBG("Closed file %s", this->file_path.c_str());
    }
    return ret;
}

int CSVLogger::sync() {
    int ret = this->sd_card->sync();
    if (ret < 0) {
        LOG_ERR("Failed to sync file: %d", ret);
    }
    return ret;
}

int CSVLogger::writeLine(const char **data, bool sync) {
    std::string line = joinLine(data, this->header_count, this->separator);
    return this->write(line, sync);
}

int CSVLogger::writeLine(const double *data, bool sync) {
    std::string line = joinLine<double>(data, this->header_count, this->separator);
    return this->write(line, sync);
}

int CSVLogger::writeLine(const int *data, bool sync) {
    std::string line = joinLine<int>(data, this->header_count, this->separator);
    return this->write(line, sync);
}

int CSVLogger::write(std::string line, bool sync) {
    line += "\n";
    size_t line_size = line.size();
    int ret = this->sd_card->write((char *)line.c_str(), &line_size);
    if (ret < 0) {
        LOG_ERR("Failed to write to file: %d", ret);
    }
    if (sync) {
        ret = this->sd_card->sync();
        if (ret < 0) {
            LOG_ERR("Failed to sync file: %d", ret);
        }
    }
    return ret;
}