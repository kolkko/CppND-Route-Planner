#include "route_planner.h"
#include <algorithm>

//RoutePlanner constructor - creates RoutePlanner object and sets its start and end node variables
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y)); // start_node is a pointer, so '&' needed to store memory address of result
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node); //* dereferences end_node, which is a pointer, so there is an actual Node to process for distance, not a memory address

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto current_neighbor : current_node->neighbors){
        current_neighbor->parent = current_node;
        current_neighbor->h_value = CalculateHValue(current_neighbor); 
        current_neighbor->g_value = current_node->g_value + current_neighbor->distance(*current_node); 
        open_list.push_back(current_neighbor);
        current_neighbor->visited = true;
    }

return;}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare(RouteModel::Node *a, RouteModel::Node *b){
    const float f1 = a->h_value + a->g_value;
    const float f2 = b->h_value + b->g_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    //std::cout << "Reached ConstructFinalPath()" << std::endl;
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node != start_node) {
        //std::cout << "In ConstructFinalPath() while loop" << std::endl;
        distance = distance + current_node->distance(*current_node->parent);
        path_found.insert(path_found.begin(), *current_node);
        current_node = current_node->parent;

    }
    //std::cout << "Exited ConstructFinalPath() while loop" << std::endl;

    path_found.insert(path_found.begin(), *start_node);


    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //std::cout << "Reached AStarSearch()" << std::endl;
    RouteModel::Node *current_node = nullptr;
    open_list.push_back(start_node);
    current_node = start_node;
    current_node->visited = true;

    // TODO: Implement your solution here.
    while(open_list.size() > 0) {
        //std::cout << "In while loop" << std::endl;

        if(current_node == end_node) {
            break; 
        }
        else {
            AddNeighbors(current_node); 
            current_node = NextNode();
        }

    }
    std::cout << "About to construct final path" << std::endl;
    m_Model.path = ConstructFinalPath(current_node); 
    std::cout << "Final path has been constructed!" << std::endl;
    return;}