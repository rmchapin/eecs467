#include <mapping/occupancy_grid.hpp>
#include <cassert>

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
/*static void update_grid()
{
	double mae_t = state->pose_time;
	double mae_x = state->pose_x_curr;
	double mae_y = state->pose_y_curr;
	double mae_th = state->pose_heading_curr;
	int num_rays = state->num_ranges;

	for (int ray = 0; ray < num_rays; ray++)
	{
		//interpolate (or extrapolate) mae_values

		double laser_t = state->lidar_times[ray];
		double laser_m = state->lidar_ranges[ray];
		double laser_th = state->lidar_thetas[ray];

		double cos_th = cos(mae_th + laser_th);
		double sin_th = cos(mae_th + laser_th);

		for (double sample_m = SAMPLE_SPACING; sample_m < laser_m; sample_m += SAMPLE_SPACING)
		{
			double sample_x = mae_x + (sample_m*cos_th);
			double sample_y = mae_y + (sample_m*sin_th);
			
			eecs467::Point<int> sample_cell = global_position_to_grid_cell(eecs467::Point<double>(sample_x, sample_y), state->grid);
			if (!state->grid.isCellInGrid(sample_cell.x, sample_cell.y)) { break; }

			int8_t odds = state->grid.logOdds(sample_cell.x, sample_cell.y);
			odds -= FREE_ADJUST_FACTOR;
			state->grid.setLogOdds(sample_cell.x, sample_cell.y, odds);
		}

		//positive adjust for ray termination cell
		double laser_term_x = mae_x + laser_m*cos_th;
		double laser_term_y = mae_y + laser_m*sin_th;
		eecs467::Point<int> laser_term_cell = global_position_to_grid_cell(eecs467::Point<double>(laser_term_x, laser_term_y), state->grid);
		int8_t odds = state->grid.logOdds(laser_term_cell.x, laser_term_cell.y);
		odds += OCCUPIED_ADJUST_FACTOR;
		state->grid.setLogOdds(laser_term_cell.x, laser_term_cell.y, odds);
	}
}*/

} // namespace eecs467
