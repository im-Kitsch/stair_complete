//
// Created by zhiyuan on 29.01.20.
//

#ifndef STAIR_COMPLETE_STAIR_COMPLETION_H
#define STAIR_COMPLETE_STAIR_COMPLETION_H

#include "parameter.h"

#include "tsdf_load_util.h"
#include "stair_mesh_util.h"
#include "stair_optimizer.h"

#include "visualizer.h"

using namespace Eigen;
using namespace std;

/*
 * Using this class for optimization process
 * It is possible to directly call "stair_completion" to do optimization,
 * otherwise is possible to use dyn_callback with dynamic reconfiguration,
 * Example -> stair_completion.cpp, offline_opt.launch
 */

class StairComplete{
public:
    Opt_Record opt_record;

    StairComplete(void){};

    int stair_completion(stair_complete::offline_dyn_paraConfig &config, Opt_Record &opt_record);

    // used for dynamic reconfiguration tools as the callback function
    int dyn_callback(stair_complete::offline_dyn_paraConfig &config, uint32_t level){
        stair_completion(config, opt_record);
    }
    
};

#endif //STAIR_COMPLETE_STAIR_COMPLETION_H
