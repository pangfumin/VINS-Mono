//
// Created by pang on 2019/10/21.
//

#ifndef VINS_ESTIMATOR_EUROCFDATAREADER_H
#define VINS_ESTIMATOR_EUROCFDATAREADER_H

#include <string>
#include <fstream>
#include <vector>

class EurocFDataReader {
public:
    EurocFDataReader(const std::string& dataset);

private:
    std::string dataset_;

};


#endif //VINS_ESTIMATOR_EUROCFDATAREADER_H
