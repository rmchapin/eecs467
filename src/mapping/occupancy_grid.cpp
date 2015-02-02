#include <mapping/occupancy_grid.hpp>
#include <cassert>
//#include "a1/maebot_data.hpp"



namespace eecs467
{

OccupancyGrid::OccupancyGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(1.0 / metersPerCell_)
, globalOrigin_(0, 0)
{
}


OccupancyGrid::OccupancyGrid(float widthInMeters,
                             float heightInMeters,
                             float metersPerCell)
: metersPerCell_(metersPerCell)
, globalOrigin_(-widthInMeters/2.0f, -heightInMeters/2.0f)
{
    assert(widthInMeters  > 0.0f);
    assert(heightInMeters > 0.0f);
    assert(metersPerCell_ <= widthInMeters);
    assert(metersPerCell_ <= heightInMeters);
    
    cellsPerMeter_ = 1.0f / metersPerCell_;
    width_         = widthInMeters * cellsPerMeter_;
    height_        = heightInMeters * cellsPerMeter_;
    
    cells_.resize(width_ * height_);
    reset();
}


void OccupancyGrid::reset(void)
{
    std::fill(cells_.begin(), cells_.end(), 0);
}


bool OccupancyGrid::isCellInGrid(int x, int y) const
{ 
    bool xCoordIsValid = (x >= 0) && (static_cast<std::size_t>(x) < width_);
    bool yCoordIsValid = (y >= 0) && (static_cast<std::size_t>(y) < height_);
    return xCoordIsValid && yCoordIsValid;
}


CellOdds OccupancyGrid::logOdds(int x, int y) const
{
    if(isCellInGrid(x, y))
    {
        return operator()(x, y);
    }
    
    return 0;
}


void OccupancyGrid::setLogOdds(int x, int y, CellOdds value)
{
    if(isCellInGrid(x, y))
    {
        operator()(x, y) = value;
    }
}


maebot_occupancy_grid_t OccupancyGrid::toLCM(void) const
{
    maebot_occupancy_grid_t grid;

    grid.origin_x        = globalOrigin_.x;
    grid.origin_y        = globalOrigin_.y;
    grid.meters_per_cell = metersPerCell_;
    grid.width           = width_;
    grid.height          = height_;
    grid.num_cells       = cells_.size();
    grid.cells           = cells_;
    
    return grid;
}


void OccupancyGrid::fromLCM(const maebot_occupancy_grid_t& gridMessage)
{
    globalOrigin_.x = gridMessage.origin_x;
    globalOrigin_.y = gridMessage.origin_y;
    metersPerCell_  = gridMessage.meters_per_cell;
    cellsPerMeter_  = 1.0f / gridMessage.meters_per_cell;
    height_         = gridMessage.height;
    width_          = gridMessage.width;
    cells_          = gridMessage.cells;
}



} // namespace eecs467
