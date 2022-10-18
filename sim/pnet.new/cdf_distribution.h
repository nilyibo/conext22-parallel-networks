#ifndef CDF_DISTRIBUTION_H
#define CDF_DISTRIBUTION_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <algorithm>

#include "utils.h"

template<typename T>
class CDFDistribution {
public:
    using Percentile = double;
    using value_type = typename std::enable_if<std::is_same<int, T>::value ||
                                               std::is_same<float, T>::value ||
                                               std::is_same<double, T>::value, T>::type;
    using record_type = std::pair<value_type, Percentile>;
    using record_vector_it = typename std::vector<std::pair<value_type, Percentile>>::iterator;

    CDFDistribution(std::string csv_file);

    inline std::string GetDistributionFilename() const {
        return this->csv_file_;
    }

    T GetOne() const;

private:
    std::string csv_file_;
    std::ifstream ifstream_;

    // An ordered list of (value, percentile)
    std::vector<record_type> distribution_;

    void ParseCsvFile();
};

template<typename T>
CDFDistribution<T>::CDFDistribution(std::string csv_file) : csv_file_(csv_file), ifstream_(std::ifstream(csv_file)) {
    CHECK(ifstream_.is_open(), "Failed to open CSV file.");
    this->ParseCsvFile();
}

template<typename T>
T CDFDistribution<T>::GetOne() const {
    double random_percentile = GetRandomNumber(1.0);
    auto it_pair = std::lower_bound(distribution_.begin(), distribution_.end(), random_percentile, [](const record_type &r, const double &percentile) {
        return r.second < percentile;
    });
    // Use the lowest if not found, i.e. percentile is too low.
    if (it_pair == distribution_.end())
        it_pair = distribution_.begin();
    return it_pair->first;
}

template<typename T>
void CDFDistribution<T>::ParseCsvFile() {
    string line;
    value_type value;
    Percentile percentile;

    CHECK(getline(ifstream_, line) && line == "Value,Percentile", "Mismatch on header line.");
    distribution_.clear();
    while (getline(ifstream_, line)) {
        std::stringstream ss(line);
        std::string item;

        std::getline(ss, item, ',');
        if (std::is_same<T, int>::value)
            value = std::stoi(item);
        else if (std::is_same<T, float>::value)
            value = std::stof(item);
        else if (std::is_same<T, double>::value)
            value = std::stod(item);
        else
            NO_IMPL;

        std::getline(ss, item);
        percentile = std::stod(item);
        auto pair = std::make_pair(value, percentile);
        distribution_.push_back(pair);
    }

    // Keep the array sorted on percentile.
    std::sort(distribution_.begin(), distribution_.end(), [](const record_type &a, const record_type &b) {
        return a.second < b.second;
    });
}

#endif  // CDF_DISTRIBUTION_H