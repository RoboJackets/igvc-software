#include "Graph.h"

Graph::Graph()
{

}

Graph::~Graph()
{

}

void Graph::setCSpace(float CSpace)
{
    this->CSpace = CSpace;
}

void Graph::setGoal(std::tuple<int,int> Goal)
{
    this->Goal.setIndex(Goal);
}

void Graph::initializeGraph(const igvc_msgs::mapConstPtr& msg)
{

    this->length = msg->length;
    this->width = msg->width;
    this->Resolution = msg->resolution;
    this->Start.setIndex(msg->x, msg->y);

    // set the current map equal to the input message's map
    Map = cv_bridge::toCvCopy(msg->image, "mono8");
}

void Graph::updateGraph(igvc_msgs::mapConstPtr& msg)
{
    // Update the start position and K_M
    std::tuple<float,float> newStart = std::make_tuple(static_cast<float>(msg->x), static_cast<float>(msg->y));
    std::tuple<float,float> oldStart = std::make_tuple(static_cast<float>(std::get<0>(this->Start.getIndex())), \
                                                       static_cast<float>(std::get<1>(this->Start.getIndex())));

    // update the heuristic adjustment value to account for the robot's new pos
    this->K_M += igvc::get_distance(oldStart, newStart);
    this->Start.setIndex(std::make_tuple(static_cast<int>(msg->x), static_cast<int>(msg->y)));

    this->updatedCells.clear();

    // get the most recently observed map (occupancy grid)
    cv_bridge::CvImagePtr currMap = cv_bridge::toCvCopy(msg->image, "mono8");

    if (std::equal(Map->image.begin<uchar>(), Map->image.end<uchar>(), \
                currMap->image.begin<uchar>()))
    {
        return; // no cells to update
    }
    else
    {
        // current and previous occupancy grid differ
        auto start_it = Map->image.begin<uchar>();

        auto it_new = currMap->image.begin<uchar>();
        auto it_map = Map->image.begin<uchar>();

        auto it_new_end = currMap->image.end<uchar>();
        auto it_map_end = Map->image.end<uchar>();

        // if cells differ, get (x,y) index of occupancy grid cell
        // and add it to the list of updated cells
        while ((it_new != it_new_end) && (it_map != it_map_end))
        {
            uchar curr_val = *it_new;
            uchar map_val = *it_map;

            //TODO get rid of this magic number. The point is that if current
            // map differs significantly from the previous map, then the edge
            // costs must be re-adjusted
            if (curr_val != map_val)
            {
                int pos = it_map - start_it;
                int row = pos/(this->width);
                int col = pos%(this->width);
                this->updatedCells.push_back(std::make_tuple(row,col));

                // update the value of Map with the value of the new map
                *it_map = curr_val;
            }

            it_new++;
            it_map++;
        }
    }

    std::cout << "Updated cells: " << this->updatedCells.size() << std::endl;
}

bool Graph::isValidNode(Node s)
{
    int x,y;
    std::tie(x,y) = s.getIndex();
    // with [row,col] indexing
    return (x<=length) && (y<=width) && (x>=0) && (y>=0);
}

bool Graph::isValidCell(std::tuple<int,int> ind)
{
    int x,y;
    std::tie(x,y) = ind;
    return (x<length) && (y<width) && (x>=0) && (y>=0);
}

bool Graph::isDiagonal(Node s, Node s_prime)
{
    int x1, y1;
    std::tie(x1,y1) = s.getIndex();

    int x2,y2;
    std::tie(x2,y2) = s_prime.getIndex();

    int x_diff = std::abs(x2-x1);
    int y_diff = std::abs(y2-y1);

    return (x_diff == 1) && (y_diff == 1);
}

std::vector<Node> Graph::nbrs(Node s, bool include_invalid)
{
    std::vector<Node> neighbors;
    int x,y;
    std::tie(x,y) = s.getIndex();

    // right
    Node r(x+1, y);
    if (include_invalid || isValidNode(r))
        neighbors.push_back(r);

    // top right
    Node tr(x+1, y+1);
    if (include_invalid || isValidNode(tr))
        neighbors.push_back(tr);

    // above
    Node t(x, y+1);
    if (include_invalid || isValidNode(t))
        neighbors.push_back(t);

    // top left
    Node tl(x-1, y+1);
    if (include_invalid || isValidNode(tl))
        neighbors.push_back(tl);

    // left
    Node l(x-1, y);
    if (include_invalid || isValidNode(l))
        neighbors.push_back(l);

    // bottom left
    Node bl(x-1, y-1);
    if (include_invalid || isValidNode(bl))
        neighbors.push_back(bl);

    // bottom
    Node b(x, y-1);
    if (include_invalid || isValidNode(b))
        neighbors.push_back(b);

    // bottom right
    Node br(x+1, y-1);
    if (include_invalid || isValidNode(br))
        neighbors.push_back(br);

    return neighbors;
}

Node Graph::ccknbr(Node s, Node s_prime)
{
    int x,y;
    std::tie(x,y) = s.getIndex();

    int x_prime,y_prime;
    std::tie(x_prime,y_prime) = s_prime.getIndex();

    int x_diff = x_prime - x;
    int y_diff = y_prime - y;

    Node ccknbr; // counter-clockwise neighbor

    if (x_diff == 1 && y_diff == 0)
    {
        ccknbr.setIndex(x+1,y+1);
    }
    else if (x_diff == 1 && y_diff == 1)
    {
        ccknbr.setIndex(x,y+1);
    }
    else if (x_diff == 0 && y_diff == 1)
    {
        ccknbr.setIndex(x-1,y+1);
    }
    else if (x_diff == -1 && y_diff == 1)
    {
        ccknbr.setIndex(x-1,y);
    }
    else if (x_diff == -1 && y_diff == 0)
    {
        ccknbr.setIndex(x-1,y-1);
    }
    else if (x_diff == -1 && y_diff == -1)
    {
        ccknbr.setIndex(x,y-1);
    }
    else if (x_diff == 0 && y_diff == -1)
    {
        ccknbr.setIndex(x+1,y-1);
    }
    else if (x_diff == 1 && y_diff == -1)
    {
        ccknbr.setIndex(x+1,y);
    }

    // if counter-clockwise neighbor node is valid (within bounds), then return
    // it. Otherwise, return a node with validity set to false.
    if (isValidNode(ccknbr))
        return ccknbr;
    else
        return Node(false);
}

Node Graph::cknbr(Node s, Node s_prime)
{
    int x,y;
    std::tie(x,y) = s.getIndex();

    int x_prime,y_prime;
    std::tie(x_prime,y_prime) = s_prime.getIndex();

    int x_diff = x_prime - x;
    int y_diff = y_prime - y;

    Node cknbr; // clockwise neighbor

    if (x_diff == 1 && y_diff == 0)
    {
        cknbr.setIndex(x+1,y-1);
    }
    else if (x_diff == 1 && y_diff == 1)
    {
        cknbr.setIndex(x+1,y);
    }
    else if (x_diff == 0 && y_diff == 1)
    {
        cknbr.setIndex(x+1,y+1);
    }
    else if (x_diff == -1 && y_diff == 1)
    {
        cknbr.setIndex(x,y+1);
    }
    else if (x_diff == -1 && y_diff == 0)
    {
        cknbr.setIndex(x-1,y+1);
    }
    else if (x_diff == -1 && y_diff == -1)
    {
        cknbr.setIndex(x-1,y);
    }
    else if (x_diff == 0 && y_diff == -1)
    {
        cknbr.setIndex(x-1,y-1);
    }
    else if (x_diff == 1 && y_diff == -1)
    {
        cknbr.setIndex(x,y-1);
    }

    // if clockwise neighbor node is valid (within bounds), then return
    // it. Otherwise, return a node with validity set to false.
    if (isValidNode(cknbr))
        return cknbr;
    else
        return Node(false);
}

std::vector<std::tuple<Node, Node>> Graph::connbrs(Node s)
{
    // get neighbors of current node, including invalid nodes
    std::vector<Node> neighbors = nbrs(s, true);

    Node sp; // most clockwise neighbor
    Node spp;

    std::vector<std::tuple<Node,Node>> connbrs;

    // first 7 consecutive neighbor pairs
    for (size_t i = 0; i < neighbors.size() - 1; i++)
    {
        sp = neighbors[i];
        spp = neighbors[i+1];

        // if both nodes valid, make a tuple and put it at the front of the list
        if (isValidNode(sp) && isValidNode(spp))
            connbrs.push_back(std::make_tuple(sp,spp));
    }

    // last consecutive neighbor pair [s8->s1]
    sp = neighbors[neighbors.size()-1];
    spp = neighbors[0];
    if (isValidNode(sp) && isValidNode(spp))
        connbrs.push_back(std::make_tuple(sp,spp));

    return connbrs;
}

float Graph::getC(Node s, Node s_prime)
{
    // index of cell between s and s_prime. s and s_prime assumed to be
    // diagonal neighbors
    std::tuple<int,int> cellInd;
    uchar cellVal;

    int x1, y1;
    std::tie(x1,y1) = s.getIndex();

    int x2,y2;
    std::tie(x2,y2) = s_prime.getIndex();

    int x_diff = x2-x1;
    int y_diff = y2-y1;

    if ((x_diff == 1) && (y_diff == 1))
    {
        cellInd = std::make_tuple(x1,y1); // top right cell
    }
    else if ((x_diff == -1) && (y_diff == 1))
    {
        cellInd = std::make_tuple(x1-1, y1); // top left cell
    }
    else if ((x_diff == -1) && (y_diff == -1))
    {
        cellInd = std::make_tuple(x1-1, y1-1); // bottom left cell
    }
    else if ((x_diff == 1) && (y_diff == -1))
    {
        cellInd = std::make_tuple(x1, y1-1); // bottom right cell
    }

    cellVal = getValWithCSpace(cellInd);

    // #TODO get rid of magic number
    if (cellVal > 178)
    {
        // cell is occupied, infinite traversal cost
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        // cell is unuccupied, diagonal cost is euclidian distance
        return DIAGONAL_COST;
    }
}

float Graph::getB(Node s, Node s_prime)
{
    // each edge has 2 neighboring cells
    std::tuple<int,int> cellInd1;
    std::tuple<int,int> cellInd2;
    float cellVal1;
    float cellVal2;

    float maxCellVal;

    int x1, y1;
    std::tie(x1,y1) = s.getIndex();

    int x2,y2;
    std::tie(x2,y2) = s_prime.getIndex();

    int x_diff = x2-x1;
    int y_diff = y2-y1;

    if ((x_diff == 1) && (y_diff == 0))
    {
        cellInd1 = std::make_tuple(x1,y1);   // top right cell
        cellInd2 = std::make_tuple(x1,y1-1); // bottom right cell
    }
    else if ((x_diff == 0) && (y_diff == 1))
    {
        cellInd1 = std::make_tuple(x1-1, y1); // top left cell
        cellInd2 = std::make_tuple(x1, y1); // top right cell
    }
    else if ((x_diff == -1) && (y_diff == 0))
    {
        cellInd1 = std::make_tuple(x1-1,y1); // top left cell
        cellInd2 = std::make_tuple(x1-1, y1-1); // bottom left cell
    }
    else if ((x_diff == 0) && (y_diff == -1))
    {
        cellInd1 = std::make_tuple(x1-1,y1-1); // bottom left cell
        cellInd2 = std::make_tuple(x1, y1-1); // bottom right cell
    }


    cellVal1 = getValWithCSpace(cellInd1);
    cellVal2 = getValWithCSpace(cellInd2);

    maxCellVal = std::max(cellVal1, cellVal2);

    // #TODO get rid of magic number
    if (maxCellVal > 178)
    {
        // cell is occupied, infinite traversal cost
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        // cell is unuccupied, diagonal cost is euclidian distance (1)
        return EDGE_COST;
    }
}

float Graph::getValWithCSpace(std::tuple<int,int> ind)
{
    float cost;

    if (isValidCell(ind))
    {
        int x,y;
        std::tie(x,y) = ind;

        int sep = CSpace/Resolution + 1;

        cv::Mat subsection =
          Map->image(cv::Range(std::max(x - sep, 0), std::min(x + sep + 1, Map->image.size().height)),
                     cv::Range(std::max(y - sep, 0), std::min(y + sep + 1, Map->image.size().width)));

         double min_val;
         double max_val;
         cv::minMaxLoc(subsection, &min_val, &max_val);

         cost = static_cast<float>(max_val);
    }
    else
    {
        cost = (float) 255;
    }

    return cost;
}

float Graph::getTraversalCost(Node s, Node s_prime)
{
    float cost;
    if (isDiagonal(s, s_prime))
        cost = getC(s, s_prime);
    else
        cost = getB(s, s_prime);

    return cost;
}

float Graph::euclidian_heuristic(Node s)
{
    std::tuple<float,float> start_f = Start.getIndex();
    std::tuple<float,float> s_f = s.getIndex();
    return igvc::get_distance(start_f, s_f);
}

float Graph::euclidian_heuristic(std::tuple<int,int> ind)
{
    std::tuple<float,float> start_f = Start.getIndex();
    std::tuple<float,float> s_f = ind;

    return igvc::get_distance(start_f, s_f);
}

std::vector<Node> Graph::getNodesAroundCellWithCSpace(std::tuple<int,int> cellInd)
{
    int x,y;
    std::tie(x,y) = cellInd;

    int sep = (CSpace/Resolution) + 1;

    std::vector<Node> toUpdate;

    for (int new_x = x - sep; new_x <= x + sep; new_x++)
    {
        for (int new_y = y - sep; new_y <= y + sep; new_y++)
        {
            Node updatedNode = Node(new_x, new_y);
            if (isValidNode(updatedNode))
                toUpdate.push_back(updatedNode);
        }
    }

    return toUpdate;
}
