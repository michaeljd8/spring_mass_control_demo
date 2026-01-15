#ifndef SAVE_TO_CSV_HPP
#define SAVE_TO_CSV_HPP

#include <fstream>
#include <string>
#include <vector>
#include <utility>

/**
 * @brief Save a vector of pairs to a CSV file (e.g., distance, velocity)
 * @param data Vector of pairs to save
 * @param filename Output filename
 * @param header Optional header line (e.g., "distance,velocity")
 * @return true if successful, false otherwise
 */
inline bool save_pairs_to_csv(const std::vector<std::pair<double, double>>& data,
                               const std::string& filename,
                               const std::string& header = "") {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    if (!header.empty()) {
        file << header << "\n";
    }
    
    for (const auto& [first, second] : data) {
        file << first << "," << second << "\n";
    }
    
    return true;
}

/**
 * @brief Save multiple columns to a CSV file
 * @param columns Vector of column data (each inner vector is a column)
 * @param filename Output filename
 * @param header Optional header line (e.g., "time,position,velocity")
 * @return true if successful, false otherwise
 */
inline bool save_columns_to_csv(const std::vector<std::vector<double>>& columns,
                                 const std::string& filename,
                                 const std::string& header = "") {
    if (columns.empty()) {
        return false;
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    if (!header.empty()) {
        file << header << "\n";
    }
    
    size_t num_rows = columns[0].size();
    for (size_t row = 0; row < num_rows; ++row) {
        for (size_t col = 0; col < columns.size(); ++col) {
            if (col > 0) file << ",";
            file << columns[col][row];
        }
        file << "\n";
    }
    
    return true;
}

#endif // SAVE_TO_CSV_HPP
