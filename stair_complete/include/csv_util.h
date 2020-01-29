//
// Created by zhiyuan on 11.12.19.
//

#ifndef STAIR_COMPLETE_CSV_UTIL_H
#define STAIR_COMPLETE_CSV_UTIL_H
#include <Eigen/Dense>
#include <vector>
#include <fstream>

using namespace Eigen;

/*
 * functions about .csv files
 * read write .csv file
 * found in stackflow
 */

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

// define the format you want, you only need one instance of this...
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void writeToCSVfile(string name, MatrixXd matrix)
{
    ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}

#endif //STAIR_COMPLETE_CSV_UTIL_H
