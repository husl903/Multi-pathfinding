#include <libMultiRobotPlanning/jpst_gridmap.hpp>
#include <algorithm>

libMultiRobotPlanning::jpst_gridmap::jpst_gridmap(gridmap* gm)
{
    gm_ = gm;
    // sipp_map_ = new warthog::sipp_gridmap(gm);

    // one copy of the map for jumping E<->W; one copy for jumping N<->S
    t_gm_ = new libMultiRobotPlanning::gridmap(gm_->header_height(), gm_->header_width());
}

libMultiRobotPlanning::jpst_gridmap::~jpst_gridmap()
{
    delete t_gm_;
    // delete gm_;
//    delete sipp_map_;
}
