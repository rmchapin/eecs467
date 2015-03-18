#include "Board.hpp"

std::string COLOR_NAMES[] = {"GREEN", "RED", "YELLOW"};

Board::Board(ImageProcessor *ip_, bool red) : ip(ip_)
{
    playerColor = (red ? RED : GREEN);
}

Board::~Board()
{
}

void Board::getCalibration(coord *data)
{
    std::ifstream input("calibration.txt");
    double x, y;
    for(int i = 0; i < 3; i++)
    {
        input >> x >> y;
        data[i] = {x, y};
    }
}

bool Board::withinBounds(int p1, int p2)
{
    return p1 < p2+50 && p1 > p2-50;
}

int Board::getCornersIndex(int x, int y, coord *calibrationData)
{
    for(int i = 0; i < 3; i++)
    {
        if(withinBounds(x, calibrationData[i].x) && withinBounds(y, calibrationData[i].y))
        {
            return i;
        }
    }
    return 3;
}

void Board::getInput(coord *calibrationData, coord *corners, std::vector<coord>& greenBalls, std::vector<coord>& redBalls, std::string filename)
{
    std::ifstream input(filename.c_str());
    std::string elt;
    gsl_vector *world;
    world = gsl_vector_alloc(3);
    gsl_vector *point;
    point = gsl_vector_alloc(3);
    int mode_switch = 0;
    
    input >> elt; // get first '#'
    while(input >> elt)
    {
        if(elt == "#")
        {
            mode_switch++;
            continue;
        }
        double x = atoi(elt.c_str());
        input >> elt;
        double y = atoi(elt.c_str());
    
        gsl_vector_set(point, 0, x);
        gsl_vector_set(point, 1, y);
        gsl_vector_set(point, 2, 1);
        ip->calculate_arm_coords(point, world);
        if(mode_switch == 0) // corners
        {
            std::cout << "corner: " << x << ", " << y << std::endl;
            int cornersIndex = getCornersIndex(x, y, calibrationData);
            std::cout << "corners index: " << cornersIndex << std::endl;
            corners[cornersIndex] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            //corners[cornersIndex] = {x, y};
        }
        else if(mode_switch == 1) // green balls
        {
			coord nextCoord = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            greenBalls.push_back(nextCoord);
            //greenBalls[index] = {x, y};
        }
        else // mode_switch == 2 : red balls
        {
			coord nextCoord = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            redBalls.push_back(nextCoord);
            //redBalls[index] = {x, y};
        }
    }
}

//void Board::boardInit(coord balls_[5], coord pos1, coord pos2, coord pos3, coord pos4, std::string color)
void Board::boardInit(std::string filename)
{
    // grab calibration data
    coord calibrationData[3];
    getCalibration(calibrationData);
    
    // get positions from file
    coord corners[4];
    std::vector<coord> greenBalls;
    std::vector<coord> redBalls;
    getInput(calibrationData, corners, greenBalls, redBalls, filename);

    // figure out x and y coords of each square
    double boardx2 = (corners[1].x + corners[0].x)/2;
    double boardx1 = (corners[1].x + boardx2)/2;
    double boardx3 = (corners[0].x + boardx2)/2;

    double boardy2 = (corners[1].y + corners[2].y)/2;
    double boardy1 = (corners[1].y + boardy2)/2;
    double boardy3 = (corners[2].y + boardy2)/2;
	
	std::cout << "boardx1: " << boardx1 << std::endl;
	std::cout << "boardx2: " << boardx2 << std::endl;
	std::cout << "boardx3: " << boardx3 << std::endl;
	std::cout << "boardy1: " << boardy1 << std::endl;
	std::cout << "boardy2: " << boardy2 << std::endl;
	std::cout << "boardy3: " << boardy3 << std::endl;

    int offset = 20;
    coord min0 = {boardx1-offset, boardy1-offset};
    coord max0 = {boardx1+offset, boardy1+offset};
    board[0] = {YELLOW, min0, max0};

    coord min1 = {boardx1-offset, boardy2-offset};
    coord max1 = {boardx1+offset, boardy2+offset};
    board[1] = {YELLOW, min1, max1};

    coord min2 = {boardx1-offset, boardy3-offset};
    coord max2 = {boardx1+offset, boardy3+offset};
    board[2] = {YELLOW, min2, max2};

    coord min3 = {boardx2-offset, boardy1-offset};
    coord max3 = {boardx2+offset, boardy1+offset};
    board[3] = {YELLOW, min3, max3};

    coord min4 = {boardx2-offset, boardy2-offset};
    coord max4 = {boardx2+offset, boardy2+offset};
    board[4] = {YELLOW, min4, max4};

    coord min5 = {boardx2-offset, boardy3-offset};
    coord max5 = {boardx2+offset, boardy3+offset};
    board[5] = {YELLOW, min5, max5};

    coord min6 = {boardx3-offset, boardy1-offset};
    coord max6 = {boardx3+offset, boardy1+offset};
    board[6] = {YELLOW, min6, max6};

    coord min7 = {boardx3-offset, boardy2-offset};
    coord max7 = {boardx3+offset, boardy2+offset};
    board[7] = {YELLOW, min7, max7};

    coord min8 = {boardx3-offset, boardy3-offset};
    coord max8 = {boardx3+offset, boardy3+offset};
    board[8] = {YELLOW, min8, max8};

    if(playerColor == GREEN)
    {
        for(uint i = 0; i < greenBalls.size(); i++)
        {
			Ball nextBall = {GREEN, greenBalls[i]};
            freeBalls.push_back(nextBall);
        }
    }
    else // if playerColor == "RED"
    {
        for(uint i = 0; i < redBalls.size(); i++)
        {
			Ball nextBall = {RED, redBalls[i]};
            freeBalls.push_back(nextBall);
        }
    }

    numFreeBalls = 5;
}

void Board::updateBoard(std::string filename)
{
    // clear board
    clearBoard();
    freeBalls.clear();
    
    // get positions from file
    std::vector<coord> greenBalls;
    std::vector<coord> redBalls;
    getBalls(greenBalls, redBalls, filename);
    
    // update red balls
    for(uint i = 0; i < redBalls.size(); i++)
    {
        int board_index = findBoardIndex(redBalls[i]);
        if(board_index != -1)
        {
            board[board_index].ball = RED;
        }
        else
        {
            if(playerColor == RED)
            {
				Ball nextBall = {RED, redBalls[i]};
				freeBalls.push_back(nextBall);
            }
        }
    }

    // update green balls
    for(uint i = 0; i < greenBalls.size(); i++)
    {
        int board_index = findBoardIndex(greenBalls[i]);
        if(board_index != -1)
        {
            board[board_index].ball = GREEN;
        }
        else
        {
            if(playerColor == GREEN)
            {
				Ball nextBall = {GREEN, greenBalls[i]};
				freeBalls.push_back(nextBall);
            }
        }
    }

    // update numFreeBalls
	numFreeBalls = ((playerColor == GREEN) ? greenBalls.size() : redBalls.size());
}

void Board::getBalls(std::vector<coord>& greenBalls, std::vector<coord>& redBalls, std::string filename)
{
    std::ifstream input(filename.c_str());
    std::string elt;
    gsl_vector *world;
    world = gsl_vector_alloc(3);
    gsl_vector *point;
    point = gsl_vector_alloc(3);
    int mode_switch = 0;
    
    input >> elt; // get first '#'
    while(input >> elt)
    {
        if(elt == "#")
        {
            mode_switch++;
            continue;
        }
        if(mode_switch == 0) // corners
        {
            continue;
        }
        double x = atoi(elt.c_str());
        input >> elt;
        double y = atoi(elt.c_str());
    
        gsl_vector_set(point, 0, x);
        gsl_vector_set(point, 1, y);
        gsl_vector_set(point, 2, 1);
        ip->calculate_arm_coords(point, world);
        if(mode_switch == 1) // green balls
        {
	        coord nextCoord = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            greenBalls.push_back(nextCoord);
            //greenBalls[index] = {x, y};
        }
        else // mode_switch == 2 : red balls
        {
		    coord nextCoord = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            redBalls.push_back(nextCoord);
            //redBalls[index] = {x, y};
        }
    }
}

bool Board::gameOver()
{
    if (numFreeBalls == 0)
    {
        std::cout << "no free balls!" << std::endl;
        return true;
    }

    for (int g = 0; g < 3; g++)
    {
        bool hor = true, vert = true;
        for (int w = 0; w < 3; w++)
        {
            if (board[3*w + g].ball != RED)
                vert = false;
            if (board[3*g + w].ball != RED)
                hor = false;
        }
        if (hor || vert)
            return true;
    }

    bool diag1 = true, diag2 = true;
    for (int d = 0; d < 3; d++)
    {
        if (board[4*d].ball != RED)
            diag1 = false;
        if (board[6 - 2*d].ball != RED)
            diag2 = false;
    }

    bool red_wins = (diag1 || diag2);

    for (int g = 0; g < 3; g++)
    {
        bool thor = true, tvert = true;
        for (int w = 0; w < 3; w++)
        {
            if (board[3*w + g].ball != GREEN)
                tvert = false;
            if (board[3*g + w].ball != GREEN)
                thor = false;
        }
        if (thor || tvert)
            return true;
    }

    bool tdiag1 = true, tdiag2 = true;
    for (int d = 0; d < 3; d++)
    {
        if (board[4*d].ball != GREEN)
            tdiag1 = false;
        if (board[6 - 2*d].ball != GREEN)
            tdiag2 = false;
    }

    bool green_wins = (tdiag1 || tdiag2);

    return (red_wins || green_wins);
}

void Board::clearBoard()
{
    for(int i = 0; i < 9; i++)
    {
        board[i].ball = YELLOW;
    }
}

int Board::findBoardIndex(coord ball)
{
    for(int i = 0; i < 9; i++)
    {
        if(ball.x > board[i].min_coord.x &&
           ball.x < board[i].max_coord.x &&
           ball.y > board[i].min_coord.y &&
           ball.y < board[i].max_coord.y)
        {
            return i;
        }
    }
    return -1;
}

std::vector<Ball> Board::getFreeBalls()
{
    return freeBalls;
}

void Board::printInit()
{
    for(int i = 0; i < 9; i++)
    {
        std::cout << "(" << board[i].min_coord.x << ", " << board[i].min_coord.y << ") -> ("
                  << board[i].max_coord.x << ", " << board[i].max_coord.y << ")    ";
        if(i%3 == 2)
        {
            std::cout << std::endl;
        }
    }
}

void Board::print()
{
    // print board
    for(int i = 0, j = 0; i < 9; i++)
    {
        if(board[i].ball == GREEN) std::cout << "G ";
        else if(board[i].ball == RED) std::cout << "R ";
        else std::cout << "  ";
        if(i%3 != 2) std::cout << "| ";
        if(i%3 == 2){
            j++;
            std::cout << std::endl;
        }
        if(j < 3 && i%3 == 2) std::cout << "--+---+--" << std::endl;
    }

    // print free balls
    for(int i = 0; i < numFreeBalls; i++)
    {
        std::cout << COLOR_NAMES[freeBalls[i].color] << " freeBalls[" << i << "]: (" << freeBalls[i].position.x << ", " << freeBalls[i].position.y << ")" << std::endl;
    }
}

coord Board::nextPick()
{
	std::cout << "pick up ball at " << freeBalls[0].position.x << ", " << freeBalls[0].position.y << std::endl;
	return freeBalls[0].position;
}

inline int Board::isCorner(int sq)
{
    if ((sq == 0) || (sq == 2) || (sq == 6) || (sq == 8))
        return 1;
    else
        return 0;
}

inline int Board::isCenter(int sq)
{
    if (sq == 4)
        return 2;
    else
        return 0;
}

int Board::isWin(int sq)
{
    board[sq].ball = playerColor;
    int ret = (gameOver() ? 8 : 0);
    board[sq].ball = YELLOW;
    return ret;
}

int Board::blocksWin(int sq)
{
    if (playerColor == RED)
        board[sq].ball = GREEN;
    else
        board[sq].ball = RED;
    int ret = (gameOver() ? 4 : 0);
    board[sq].ball = YELLOW;
    return ret;
}

coord Board::nextPlace()
{
    int max_value = -1;
    int choose = -1;
    for (int f = 0; f < 9; f++)
    {   
        int value = 0;
        if (board[f].ball == YELLOW) //if available
        {
            value += isWin(f);
            value += blocksWin(f);
            value += isCenter(f);
            value += isCorner(f);

            if (value > max_value)
            {
                max_value = value;
                choose = f;
            }
        }
    }

    if (max_value == -1)
    {
        std::cout << "game has no valid moves!" << std::endl;
        exit(-4);
    }
	else
	{
		std::cout << "place ball at " << choose << std::endl;
	}

    //coord is avg of min, max of square
    coord ret;
    ret.x = 0.5*(board[choose].min_coord.x + board[choose].max_coord.x);
    ret.y = 0.5*(board[choose].min_coord.y + board[choose].max_coord.y);
    return ret;
}
