#include <bits/stdc++.h>
using namespace std::chrono;
using namespace std;
const int INF = INT_MAX/2;
int map_width;         // change it one time to the width of the map.
int map_height;        // change it one time to the height of the map.
int size_all_nodes, map_size, number_of_free_cells;
int *heuristic, *hmax;
//Classes with operators:
struct Edge
{
    int cell_id_, height_;
    Edge(int cell_id, int height) : cell_id_(cell_id), height_(height) {}
    bool operator==(const Edge &other) const
    {
        return cell_id_ == other.cell_id_ && height_ == other.height_;
    }
};
bool operator<(Edge a, Edge b)
{
    return a.height_ != b.height_ ? a.height_ < b.height_ : a.cell_id_ < b.cell_id_;
}
namespace std
{
    template <>
    struct hash<Edge>
    {
        auto operator()(const Edge &e) const -> size_t
        {
            return e.height_ * size_all_nodes + e.cell_id_;
        }
    };
}
struct Interval
{
    int l_, h_, cost_;
    Interval() {}
    Interval(int l, int h) : l_(l), h_(h) { cost_ = l; }
    Interval(int l, int h, int cost) : l_(l), h_(h), cost_(cost) {}
};
struct HeightCostPair
{
    int height_, cost_;
    HeightCostPair() {}
    HeightCostPair(int height, int cost) : height_(height), cost_(cost) {}
};
bool operator < (HeightCostPair a, HeightCostPair b)
{
    return a.height_ != b.height_ ? a.height_ < b.height_ : a.cost_ < b.cost_;
}
bool operator == (HeightCostPair a, HeightCostPair b)
{
    return a.height_ == b.height_ && a.cost_ == b.cost_;
}
struct Node
{
    int parent_, cell_id_, h_cost_;
    HeightCostPair height_cost_pair_;
    Node() {}
    Node(HeightCostPair height_cost_pair, int parent, int cell_id) : height_cost_pair_(height_cost_pair), parent_(parent), cell_id_(cell_id) {
        if(cell_id<number_of_free_cells){ // a layer-node
            h_cost_ = heuristic[cell_id] + height_cost_pair.height_/2*2+1;
        }
        else{ // non-layer node
            h_cost_ = 0;
        }
    }
};
bool operator == (Node a, Node b){
    return a.cell_id_ == b.cell_id_ && a.height_cost_pair_ == b.height_cost_pair_;
}


// storage::
bool *map_cell; // cell=map_cell[id]: true -> obstacle, false -> free
int *free_cells;
vector<int> *neighbors;
int sink_id;
vector<int> start_cells, goal_cells;
bool *is_goal_cell;
// The edges will be stored in the 'edges[destination]'.
set<int> hub_to_goal_edges;  // To the goal, it is enough to store the id of the hub nodes which are connected to goal.
set<Edge> *edges; // between other cells, it is necessary to store the time step and the id of the next cell.
set<HeightCostPair> *visited_heights;
set<int> *blocks;
int *min_cost;


void init1(int map_size){
    map_cell = new bool [map_size]();
    free_cells = new int [map_size]();
}
void init2(int num_of_free_cells, int num_of_goals){
    neighbors = new vector<int> [num_of_free_cells]();
    is_goal_cell = new bool [num_of_free_cells]();
    edges = new set<Edge> [num_of_free_cells+num_of_goals+1]();
    visited_heights = new set<HeightCostPair> [num_of_free_cells+num_of_goals+1]();
    blocks = new set<int> [num_of_free_cells +num_of_goals+1]();
    min_cost = new int [num_of_free_cells +num_of_goals + 1]();
    heuristic = new int [num_of_free_cells]();
    hmax = new int [num_of_free_cells]();
}
void del(){
    delete []map_cell;
    delete []free_cells;
    delete []neighbors;
    delete []is_goal_cell;
    delete []edges;
    delete []visited_heights;
    delete []blocks;
    delete []min_cost;
    delete []heuristic;
    delete []hmax;
}

inline bool isSink(int cell_id)
{
    return cell_id == sink_id;
}
inline bool isHub(int cell_id)
{
    return cell_id >= number_of_free_cells && cell_id < sink_id;
}
inline bool isHubOrSink(int cell_id){
    return cell_id >= number_of_free_cells;
}
bool operator<(const Node &a, const Node &b)
{ // inverted because of use of priority_queue
    return a.height_cost_pair_.cost_+a.h_cost_ != b.height_cost_pair_.cost_+b.h_cost_? 
    a.height_cost_pair_.cost_+a.h_cost_ > b.height_cost_pair_.cost_+b.h_cost_ :
    (
        a.height_cost_pair_.cost_ != b.height_cost_pair_.cost_?
        a.height_cost_pair_.cost_ < b.height_cost_pair_.cost_:
        (
            a.height_cost_pair_.height_ > b.height_cost_pair_.height_
        )
        
    );
}
void compressFreeCells(int num_of_goals){
    number_of_free_cells = 0;
    int cnt = 0;
    for (int i = 0; i < map_height; ++i)
    {
        for (int j = 0; j < map_width; ++j)
        {
            if (!map_cell[cnt])
            {
                free_cells[cnt] = number_of_free_cells++;
            }
            ++cnt;
        }
    }
    size_all_nodes = number_of_free_cells+num_of_goals+1;
    init2(number_of_free_cells, num_of_goals);
}
void genAllNeighbors(int num_of_goals)
{ // fill 'map' and 'is_goal_cell' sequentially, 'map_width' and 'map_height' and you are ready to call this function.
    int cnt = 0, cnt_hub_nodes = number_of_free_cells;
    sink_id = number_of_free_cells + num_of_goals;
    int di[4] = {0, 0, 1, -1}, dj[4] = {1, -1, 0, 0};
    for (int i = 0; i < map_height; ++i)
    {
        for (int j = 0; j < map_width; ++j)
        {
            if (!map_cell[cnt])
            {
                int cell = free_cells[cnt];
                for (int k = 0; k < 4; ++k)
                {
                    int ii = i + di[k];
                    int jj = j + dj[k];
                    int id = cnt + dj[k] + di[k] * map_width;
                    if (ii < map_height && ii >= 0 && jj < map_width && jj >= 0)
                    {
                        if (!map_cell[id])
                        {
                            neighbors[cell].push_back(free_cells[id]);
                        }
                    }
                }
                if (is_goal_cell[cell])
                {
                    neighbors[cell].push_back(cnt_hub_nodes++);
                }
            }
            ++cnt;
        }
    }
}
pair<bool, Interval> checkHeightAndGetUnvisitedSafeInterval(HeightCostPair height_cost_pair, int cell_id)
{
    if (isHub(cell_id))
    {
        if (min_cost[cell_id] <= height_cost_pair.cost_)
        {
            return {false, Interval()};
        }
        return {true, Interval(height_cost_pair.height_, height_cost_pair.height_, height_cost_pair.cost_)};
    }
    if (isSink(cell_id))
    {
        if (min_cost[cell_id] < height_cost_pair.cost_)
        {
            return {false, Interval()};
        }
        return {true, Interval(height_cost_pair.height_, height_cost_pair.height_, height_cost_pair.cost_)};
    }
    int prev_block = -1, nxt_block = INF;
    auto itr = blocks[cell_id].lower_bound(height_cost_pair.height_);
    if (itr != blocks[cell_id].end())
    {
        nxt_block = *itr;
    }
    if (itr != blocks[cell_id].begin())
    {
        --itr;
        prev_block = *itr;
    }
    auto itrt = visited_heights[cell_id].upper_bound(HeightCostPair(height_cost_pair.height_, INF + 10));
    while (itrt != visited_heights[cell_id].begin())
    {
        --itrt;
        // assert(itrt->height_ <= height_cost_pair.height_);
        if (itrt->height_ > prev_block)
        {
            if(itrt->cost_ <= height_cost_pair.cost_)
            {
                return make_pair(false, Interval());
            }
            else if(nxt_block==INF && itrt->cost_+itrt->height_+max(0,hmax[cell_id]
            -itrt->height_) <= height_cost_pair.cost_+height_cost_pair.height_){
                return make_pair(false, Interval());
            }
        }
        else
            break;
    }
    if(height_cost_pair.height_>=hmax[cell_id]){
        return {true, Interval(height_cost_pair.height_, height_cost_pair.height_+1, height_cost_pair.cost_)}; 
    }
    int nxt_visited_height = hmax[cell_id]+2;
    while (itrt != visited_heights[cell_id].end())
    {
        if (itrt->height_ > nxt_block)
        {
            break;
        }
        if (itrt->height_ >= height_cost_pair.height_ && itrt->cost_ <= height_cost_pair.cost_)
        {
            nxt_visited_height = itrt->height_;
            break;
        }
        ++itrt;
    }
    if (nxt_visited_height < nxt_block)
    {
        return {true, Interval(height_cost_pair.height_, nxt_visited_height - 1, height_cost_pair.cost_)};
    }
    return {true, Interval(height_cost_pair.height_, nxt_block, height_cost_pair.cost_)};
}
bool checkHeight(HeightCostPair height_cost_pair, int cell_id)
{
    if (isHub(cell_id))
    {
        if (min_cost[cell_id] <= height_cost_pair.cost_)
        {
            return false;
        }
        return true;
    }
    if (isSink(cell_id))
    {
        if (min_cost[cell_id] < height_cost_pair.cost_)
        {
            return false;
        }
        return true;
    }
    int prev_block = -1, nxt_block = INF;
    auto itr = blocks[cell_id].lower_bound(height_cost_pair.height_);
    if (itr != blocks[cell_id].end())
    {
        nxt_block = *itr;
    }
    if (itr != blocks[cell_id].begin())
    {
        --itr;
        prev_block = *itr;
    }
    auto itrt = visited_heights[cell_id].upper_bound(HeightCostPair(height_cost_pair.height_, INF + 10));
    while (itrt != visited_heights[cell_id].begin()) // iterate over all states in the same Bulk and less height.
    {
        --itrt;
        // assert(itrt->height_ <= height_cost_pair.height_);
        if (itrt->height_ > prev_block)
        {
            if(itrt->cost_ <= height_cost_pair.cost_)
            {
                return false;
            }
            else if(nxt_block==INF && itrt->cost_+itrt->height_+max(0,hmax[cell_id]-itrt->height_) <= height_cost_pair.cost_+height_cost_pair.height_){
                return false;
            }
        }
        else // if(itrt->height_ <= prev_block)
        {
            break;
        }
    }
    return true;
}
bool isThereAnEdgeToGoalFromHubNode(int cell_id) // check if the edge from this hub node to sink is available.
{
    return hub_to_goal_edges.count(cell_id);
}
bool isThereAnEdge(int source, int destination, int height) // used to check the reversed edges.
{
    return edges[destination].count(Edge(source, height));
}
void visit(int cell_id, HeightCostPair height_cost_pair) // CLOSED is not one set, but the visited states are stored in more efficient way.
{
    if (isHub(cell_id) || isSink(cell_id))
    {
        min_cost[cell_id] = min(min_cost[cell_id], height_cost_pair.cost_);
        return;
    }
    auto it = visited_heights[cell_id].insert(height_cost_pair);
    auto it2 = it.first;
    ++it2;
    auto it_nxt_block = blocks[cell_id].lower_bound(height_cost_pair.height_);
    int nxt_blocked_height;
    if (it_nxt_block == blocks[cell_id].end())
    {
        nxt_blocked_height = INF;
    }
    else
    {
        nxt_blocked_height = *it_nxt_block;
    }
    while (it2 != visited_heights[cell_id].end())
    {
        if (it2->height_ > nxt_blocked_height)
            break;
        if (it2->height_ >= height_cost_pair.height_)
        {
            if((it2->cost_ >= height_cost_pair.cost_)||(height_cost_pair.cost_ + height_cost_pair.height_ + max(0, hmax[cell_id]-height_cost_pair.height_) <= it2->height_+it2->cost_)){
                auto itTmp = it2;
                ++it2;
                visited_heights[cell_id].erase(itTmp);
            }
            else
            {
                break;
            }
        }
    }
}
void getAllHeights(int lowest_height, int hightest_height, int cell_id, vector<int> &result) // used to get the minimal heights in each connected-sequence at 'cell_id' between 'lowest_height' and 'hightest_height'.
{
    // reminder: no hub or sink nodes should go here (i.e., just layer-nodes).
    auto it_start = blocks[cell_id].lower_bound(lowest_height);
    for (auto it=it_start; it!=blocks[cell_id].end(); ++it)
    {
        if (lowest_height > hightest_height)
            break;
        if (*it >= lowest_height)
        {
            result.push_back(lowest_height);
            lowest_height = *it + 2;
        }
    }
    if (lowest_height <= hightest_height)
    {
        result.push_back(lowest_height);
    }
}
void getAllHeightCostPairs(int cell_source, int cell_destination, Interval it, vector<HeightCostPair> &result)
{
    vector<int> heights;
    int lowest_height, hightest_height;
    if (!isHub(cell_destination))
    {
        if (it.h_ % 2 == 0) // it.h_ is in the copy 'in' of the layer.
        {
            if (isThereAnEdge(cell_destination, cell_source, it.h_ - 1))
            {
                heights.push_back(it.h_ - 1);
            }
            hightest_height = it.h_;
        }
        else // it.h_ is in the copy 'out' of the layer.
        {
            if (isThereAnEdge(cell_source, cell_destination, it.h_))
            {
                hightest_height = it.h_ - 1;
            }
            else
            {
                hightest_height = it.h_ + 1;
            }
        }
        if (it.l_ % 2) // it.l_ is in the copy 'out' of the layer.
        {
            if (isThereAnEdge(cell_source, cell_destination, it.l_))
            {
                lowest_height = it.l_ + 3;
            }
            else
            {
                lowest_height = it.l_ + 1;
            }
        }
        else // it.l_ is in the copy 'in' of the layer.
        {
            if (isThereAnEdge(cell_destination, cell_source, it.l_ - 1))
                lowest_height = it.l_ -1;
            else
                lowest_height = it.l_ + 2;
        }
        if (hightest_height >= lowest_height)
        {
            getAllHeights(lowest_height, hightest_height, cell_destination, heights);
        }
        for (auto h : heights)
        {
            result.push_back(HeightCostPair(h, it.cost_));
        }
    }
    else
    {
        vector<int> heights;
        if (it.h_ % 2 == 0) // it.h_ is in the copy 'in' of the layer.
        {
            hightest_height = it.h_ - 1;
        }
        else // it.h_ is in the copy 'out' of the layer.
        {
            if (isThereAnEdge(cell_source, cell_destination, it.h_))
            {
                hightest_height = it.h_ - 2;
            }
            else
            {
                hightest_height = it.h_;
            }
        }
        if (it.l_ % 2) // it.l_ is in the copy 'out' of the layer.
        {
            if (isThereAnEdge(cell_source, cell_destination, it.l_))
            {
                lowest_height = it.l_ + 2;
            }
            else
            {
                lowest_height = it.l_;
            }
        }
        else // it.l_ is in the copy 'out' of the layer.
        {
            lowest_height = it.l_ + 1;
        }
        if (hightest_height >= lowest_height && lowest_height + it.cost_ < min_cost[cell_destination])
        {
            result.push_back(HeightCostPair(lowest_height, lowest_height + it.cost_));
        }
    }
}

vector<int> sink_nodes_id;
void addPathEdges(vector<Node> &nodes, int &number_of_remained_paths) // the second step in SSPA method.
{
    bool *marked = new bool [nodes.size()]();
    for(auto sink_node_id:sink_nodes_id){
        Node node1, node2 = nodes[sink_node_id];
        int par = node2.parent_; 
        bool flag_failed = false;
        vector<Node> path_nodes;
        path_nodes.push_back(node2);
        while (par != -1)
        {
            node1 = nodes[par];
            if(marked[par]){
                flag_failed = true;
                break;
            }
            marked[par]=true;
            if (isHub(node2.cell_id_) || isSink(node2.cell_id_))
            {
                Node node_tmp = node1;
                if (node1.height_cost_pair_.height_ < node2.height_cost_pair_.height_)
                {
                    for (int height = node2.height_cost_pair_.height_; height >= node1.height_cost_pair_.height_; height--)
                    {
                        node_tmp.height_cost_pair_.height_ = height;
                        path_nodes.push_back(node_tmp);
                    }
                }
                else
                {
                    for (int height = node2.height_cost_pair_.height_; height <= node1.height_cost_pair_.height_; height++)
                    {
                        node_tmp.height_cost_pair_.height_ = height;
                        path_nodes.push_back(node_tmp);
                    }
                }
            }
            else if (isHub(node1.cell_id_) || isSink(node1.cell_id_))
            {
                path_nodes.push_back(node1);
            }
            else
            {
                Node node_tmp = node1;
                if (node1.height_cost_pair_.height_ <= node2.height_cost_pair_.height_)
                {
                    if(node2.height_cost_pair_.height_ %2 ){ // Case when go to the upper bound of the time interval then go downward using negative edge.
                        for (int height = node2.height_cost_pair_.height_ + 1; height >= node1.height_cost_pair_.height_; height--)
                        {
                            node_tmp.height_cost_pair_.height_ = height;
                            path_nodes.push_back(node_tmp);
                        }
                    }
                    else{
                        for (int height = node2.height_cost_pair_.height_ - 1; height >= node1.height_cost_pair_.height_; height--)
                        {
                            node_tmp.height_cost_pair_.height_ = height;
                            path_nodes.push_back(node_tmp);
                        }
                    }
                }
                else
                {
                    for (int height = node2.height_cost_pair_.height_ + 1; height <= node1.height_cost_pair_.height_; height++)
                    {
                        node_tmp.height_cost_pair_.height_ = height;
                        path_nodes.push_back(node_tmp);
                    }
                }
            }
            node2 = node1;
            par = node1.parent_;
        }
        if(flag_failed)continue;
        --number_of_remained_paths;
        std::reverse(path_nodes.begin(), path_nodes.end());
        for (int i = 0; i + 1 < path_nodes.size(); ++i)
        {
            node1 = path_nodes[i];
            node2 = path_nodes[i + 1];
            if(!isHubOrSink(node1.cell_id_) && !isHubOrSink(node2.cell_id_)){
                if (node1.height_cost_pair_.height_ > node2.height_cost_pair_.height_)
                { // --> the edges is between map-cells (none-hub-none-goal nodes).
                    edges[node1.cell_id_].erase({node2.cell_id_, node2.height_cost_pair_.height_});
                    if (node1.cell_id_ == node2.cell_id_)
                    {
                        blocks[node1.cell_id_].erase(node2.height_cost_pair_.height_);
                    }
                }
                else // if (node1.height_cost_pair_.height_ < node2.height_cost_pair_.height_)
                { // --> the edge is between map-cells (none-hub-none-goal nodes).
                    edges[node2.cell_id_].insert({node1.cell_id_, node1.height_cost_pair_.height_});
                    if (node1.cell_id_ == node2.cell_id_)
                    {
                        blocks[node1.cell_id_].insert(node1.height_cost_pair_.height_);
                    }
                }
            }
            else
            {
                if (isHub(node1.cell_id_) && isSink(node2.cell_id_))
                {
                    hub_to_goal_edges.insert(node1.cell_id_);
                }
                else if (isSink(node1.cell_id_) && isHub(node2.cell_id_))
                {
                    hub_to_goal_edges.erase(node2.cell_id_);
                }
                else if (isHub(node2.cell_id_))
                { // the other node is map-cell.
                    edges[node2.cell_id_].insert({node1.cell_id_, node1.height_cost_pair_.height_});
                }
                else
                { // isHub(node1.cell_id_)
                    edges[node1.cell_id_].erase({node2.cell_id_, node2.height_cost_pair_.height_});
                }
            }
        }
    }
    delete []marked;
}

void calculateHeuristic(){
    queue<pair<int, int>> q;
    vector<bool> vis(number_of_free_cells, 0);
    for(auto node:goal_cells){
        q.push({0,node});
    }
    while(!q.empty()){
        auto tmp = q.front();
        q.pop();
        if(vis[tmp.second])continue;
        vis[tmp.second]=1;
        heuristic[tmp.second]=tmp.first;
        for(auto it:neighbors[tmp.second]){
            if(it<number_of_free_cells){
                if(!vis[it]){
                    q.push({tmp.first+2,it});
                }
            }
        }
    }
}

int cal_h_time;
void calculateHMax(){
    auto start = high_resolution_clock::now();
    priority_queue<pair<int, int>> q;
    vector<bool> vis(number_of_free_cells, 0);
    for(int i=0; i<number_of_free_cells; ++i){
        if(blocks[i].size()>0){
            q.push({(*--blocks[i].end())+2, i});
        }
    }
    while(!q.empty()){
        auto tmp = q.top();
        q.pop();
        if(vis[tmp.second])continue;
        vis[tmp.second]=1;
        hmax[tmp.second]=max(0, tmp.first);
        for(auto it:neighbors[tmp.second]){
            if(it<number_of_free_cells){
                if(!vis[it]){
                    q.push({tmp.first-2,it});
                }
            }
        }
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cal_h_time += duration.count()/1000;
}
vector<Node> nodes;
int num_of_expansions, num_of_opened;
int solve()
{
    int add_paths_duration = 0;
    int soc = 0;
    int number_of_remained_paths = start_cells.size(), past_goal_cost = 0;
    num_of_expansions = 0;
    num_of_opened = 0;
    while (number_of_remained_paths > 0) // we repeat SSPA number of times equal the number of agents.
    {
        priority_queue<Node> q;
        nodes.clear();
        for (auto it : start_cells)
        {
            q.push(Node(HeightCostPair(0, 0), -1, it));
        }
        int sink_node_id = 1, goal_cost = INF;
        for (int i = 0; i <= sink_id; ++i)
        {
            visited_heights[i].clear();
            min_cost[i] = INF;
        }
        calculateHMax(); // we need to repeat calculating hmax in every SSPA iteration.
        while (!q.empty())
        {
            Node popped_state = q.top();
            int node_id = nodes.size();
            nodes.push_back(popped_state);
            q.pop();
            int current_cell = popped_state.cell_id_;
            HeightCostPair height_cost_pair = popped_state.height_cost_pair_;
            pair<bool, Interval> res = checkHeightAndGetUnvisitedSafeInterval(height_cost_pair, current_cell);
            ++num_of_opened;
            if (!res.first)
            { // visited time step in ths cell.
                continue;
            }
            ++num_of_expansions;
            if(height_cost_pair.cost_+popped_state.h_cost_ > goal_cost)break;
            visit(current_cell, height_cost_pair);
            if (isSink(current_cell))
            {
                if(height_cost_pair.cost_ < goal_cost){
                    sink_nodes_id.clear();
                    sink_nodes_id.push_back(node_id);
                    goal_cost = height_cost_pair.cost_;
                }
                else{ //if(height_cost_pair.cost_ == goal_cost)
                    sink_nodes_id.push_back(node_id);
                }
                continue;
            }
            if (isHub(current_cell))
            {
                if (!isThereAnEdgeToGoalFromHubNode(current_cell))
                {
                    q.push(Node(height_cost_pair, node_id, sink_id));
                }
                for (auto &edge : edges[current_cell])
                {
                    q.push(Node(HeightCostPair(edge.height_, height_cost_pair.cost_-edge.height_), node_id, edge.cell_id_));
                }
                continue;
            }
            vector<int> nebs = neighbors[current_cell];
            Interval current_interval = res.second;
            // special edge: inverse of wait action.
            if (isThereAnEdge(current_cell, current_cell, current_interval.l_ - 1))
            {
                q.push(Node(HeightCostPair(current_interval.l_ - 1, height_cost_pair.cost_), node_id, current_cell));
            }

            for (auto neighbor_cell : nebs)
            {
                vector<HeightCostPair> result;
                getAllHeightCostPairs(current_cell, neighbor_cell, current_interval, result);
                for (auto it : result)
                {
                    if (checkHeight(it, neighbor_cell)){
                        q.push(Node(it, node_id, neighbor_cell));
                    }
                }
            }
        }
        if (sink_nodes_id.size() == 0)
        {
            printf("Error! Couldn't find a solution for another agent. Still remain %d goal cells.\n", number_of_remained_paths + 1);
            exit(0);
        }
        soc += goal_cost / 2 * number_of_remained_paths;
        addPathEdges(nodes, number_of_remained_paths);
        soc -= goal_cost / 2 * number_of_remained_paths;
        cerr<<"Remain "<<number_of_remained_paths<<" agents\n";
    }
    return soc;
}
void readMap(string map_name)
{
    ifstream in;
    in.open(map_name.c_str());
    string type, map_height_, map_width_, map_, row;
    in >> type >> type >> map_height_ >> map_height >> map_width_ >> map_width >> map_;
    map_size = map_height * map_width;
    init1(map_size);
    int cnt = 0;
    for (int i = 0; i < map_height; ++i)
    {
        in >> row;
        for (int j = 0; j < map_width; ++j)
        {
            if (row[j] == '@' || row[j] == 'T')
            {
                map_cell[cnt++] = 1;
            }
            else
            {
                map_cell[cnt++] = 0;
            }
        }
    }
    in.close();
}
void readInputs(string file_name, int num_of_agnets)
{
    ifstream in;
    in.open(file_name.c_str());
    string version;
    in >> version >> version;
    string num, map_;
    int w, h, r, c;
    double double_num;
    int cnt = 0;
    while (in >> num >> map_ && cnt++ < num_of_agnets)
    {
        in >> w >> h;
        assert(w == map_width);
        assert(h == map_height);
        in >> c >> r;
        start_cells.push_back(free_cells[r * map_width + c]);
        in >> c >> r;
        is_goal_cell[free_cells[r * map_width + c]] = 1;
        goal_cells.push_back(free_cells[r*map_width+c]);
        in >> double_num;
    }
    in.close();
}
int numOfAgents(string file_name){
  ifstream in(file_name.c_str());
  if(!in.good()){
    return 0;
  }
  string version;
  in>>version>>version;
  string num, map;
  int w, h, r, c;
  double double_num;
  int cnt = 0;
  while(in>>num>>map){
    in>>w>>h;
    in>>c>>r;
    in>>c>>r;
    in>>double_num;
    ++cnt;
  }
  in.close();
  return cnt;
}
void reset()
{
    start_cells.clear();
    goal_cells.clear();
    hub_to_goal_edges.clear();
    del();
}

vector<string> map_names = {
    "Berlin_1_256",
    "Boston_0_256",
    "brc202d",
    "den520d",
    "maze-128-128-10",
    "orz900d",
    "Paris_1_256",
    "warehouse-20-40-10-2-2",
    "w_woundedcoast",
    "empty-8-8",
    "empty-16-16",
    "empty-32-32",
    "ht_mansion_n",
    "maze-32-32-2",
    "maze-32-32-4",
    "maze-128-128-1",
    "maze-128-128-2",
    "lak303d",
    "lt_gallowstemplar_n",
    "ost003d",
    "random-32-32-10",
    "random-32-32-20",
    "random-64-64-20",
    "warehouse-10-20-10-2-1",
    "warehouse-20-40-10-2-1",
    "room-32-32-4",
    "room-64-64-8",
    "room-64-64-16",
    "ht_chantry",
    "warehouse-10-20-10-2-2",
    "random-64-64-10",
    "empty-48-48",
    "den312d"
};

int main()
{
    ofstream results_cout("../results/results.csv");
    results_cout << "Map,TestNum,NumOfAgents,Cost,Runtime,H_Runtime,NumOfExpansions,NumOfOpenedNodes\n";
    for (int map_id = 0; map_id < map_names.size(); ++map_id)
    {
        string map_name = map_names[map_id];
        for (int scen = 1; scen <= 25; ++scen)
        {
            int maximum_num_of_agents = numOfAgents("../data/mapf-scen-random/scen-random/" +  map_name + "-random-" + to_string(scen) + ".scen");
            for (int num_of_agnets = 1; num_of_agnets <= maximum_num_of_agents; num_of_agnets=min(maximum_num_of_agents, num_of_agnets<10?num_of_agnets+1:(num_of_agnets==10?num_of_agnets+40:num_of_agnets+50)))
            {
                readMap("../data/mapf-map/" + map_name + ".map");
                compressFreeCells(num_of_agnets);
                readInputs("../data/mapf-scen-random/scen-random/" + map_name + "-random-" + to_string(scen) + ".scen", num_of_agnets);
                genAllNeighbors(num_of_agnets);
                auto start = high_resolution_clock::now();
                calculateHeuristic();
                cal_h_time = 0;
                int soc = solve();
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                // if (duration.count() > 60000000)
                //     break;
                results_cout << map_name << "," << scen << "," << num_of_agnets << "," << soc << "," << int(duration.count() / 1000)-cal_h_time <<","<< cal_h_time<<","<<num_of_expansions<<","<<num_of_opened<<endl;
                cerr << map_name << "," << scen << "," << num_of_agnets << "," << soc << "," << int(duration.count() / 1000)-cal_h_time <<","<<cal_h_time<<","<<num_of_expansions<<","<<num_of_opened<< endl;
                if(num_of_agnets==maximum_num_of_agents)break;
                reset();
            }
            reset();
        }
    }
    results_cout.close();
}