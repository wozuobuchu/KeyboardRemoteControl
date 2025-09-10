#ifndef _PARAMS_SERVER_HPP_
#define _PARAMS_SERVER_HPP_

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <unordered_map>

class ParamsServer {
private:
    // Helper: trim from start (in place)
    static inline void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch){ return !std::isspace(ch); }));
    }
    // Helper: trim from end (in place)
    static inline void rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
    }
    // Helper: trim both ends
    static inline void trim(std::string &s) {
        ltrim(s);
        rtrim(s);
    }

    std::string cfg_file_location_;
    std::unordered_map<std::string, std::string> param_umap_;

public:
    ParamsServer() = delete;

    ParamsServer(std::string cfg_file_location) : cfg_file_location_(cfg_file_location) {
        std::ifstream ifs(cfg_file_location_);
        if (!ifs.is_open()) {
            throw std::runtime_error("Cannot open config file: " + cfg_file_location_);
        }
        std::string line;
        std::size_t line_no = 0;
        while (std::getline(ifs, line)) {
            ++line_no;
            std::string raw = line; // keep original for error messages if needed
            // trim line
            trim(line);
            // skip empty lines
            if (line.empty()) continue;
            // skip comment lines (starting with '#')
            if (!line.empty() && line[0] == '#') continue;
            // find first ':' separator
            auto pos = line.find(':');
            if (pos == std::string::npos) {
                std::ostringstream oss;
                oss << "Parse error at line " << line_no << ": missing ':' -> \"" << raw << "\"";
                throw std::runtime_error(oss.str());
            }
            std::string name = line.substr(0, pos);
            std::string value = line.substr(pos + 1); // rest after ':'
            trim(name);
            trim(value);
            if (name.empty()) {
                std::ostringstream oss;
                oss << "Parse error at line " << line_no << ": empty variable name -> \"" << raw << "\"";
                throw std::runtime_error(oss.str());
            }
            // If you prefer to allow duplicate keys to overwrite, remove this check.
            if (param_umap_.find(name) != param_umap_.end()) {
                std::ostringstream oss;
                oss << "Parse error at line " << line_no << ": duplicate variable \"" << name << "\" -> \"" << raw << "\"";
                throw std::runtime_error(oss.str());
            }
            param_umap_.emplace(std::move(name), std::move(value));
        }
    }

    template <typename T>
    bool load_param(T& var_target, std::string var_name) {
        auto it = this->param_umap_.find(var_name);
        if(it == this->param_umap_.end()) return false;

        std::stringstream ss;
        ss<<it->second;

        return static_cast<bool>(ss>>var_target);
    }

    void print_all() {
        for(auto it=this->param_umap_.begin(); it!=this->param_umap_.end(); ++it) {
            std::cout<<it->first<<": "<<it->second<<"\n";
        }
    }
};

#endif