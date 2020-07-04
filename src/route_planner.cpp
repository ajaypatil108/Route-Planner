#include "route_planner.h"
#include <algorithm>
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

  // Set start node and end node to node closest to start and end points provided by the user
	  start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// Compute h-value based on distance to end node
  float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*(this->end_node));
 }


// Expand current node by adding all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  current_node->FindNeighbors();

  // Set the parent, h_value & the g_value for each neighbor 
  for (auto &the_new_neighbor : current_node->neighbors)
  	{
    	the_new_neighbor->parent = current_node;
    	the_new_neighbor->h_value = this->CalculateHValue(the_new_neighbor);  
      // Add the neighbor to open_list and set the node's visited attribute to true
      the_new_neighbor->g_value = current_node->g_value + current_node->distance(*the_new_neighbor); 
    	the_new_neighbor->visited = true;	
    	this->open_list.push_back(the_new_neighbor);
  	}
}

bool Compare_F_Values(const RouteModel::Node* a, const RouteModel::Node* b)
{
  	float f_value_a = a->h_value + a->g_value; 
    float f_value_b = b->h_value + b->g_value;
  return f_value_a > f_value_b;
}


//  NextNode method to sort the open list according to sum of h value & g value and return the next node
RouteModel::Node *RoutePlanner::NextNode() {
  	std::sort(open_list.begin(), open_list.end(), Compare_F_Values);
    RouteModel::Node* next_node = open_list.back();
  	open_list.pop_back();
  	return next_node;
}


// ConstructFinalPath method to return the final path found from A* search
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  	while(current_node->parent != nullptr) {
      path_found.push_back(*current_node);
      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;	
    }
  	path_found.push_back(*current_node); //Pushes back the starting node
  
 	  std::reverse(path_found.begin(),path_found.end()); //Reverses order of nodes in path_found

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
  	start_node->visited = true; // Since we are starting at the start node
    open_list = {start_node};	// Initializing the open list with start node for the Search algorithm to start
  
  	RouteModel::Node *current_node = nullptr; 
    // TODO: Implement your solution here.
    while(open_list.size()>0) {   
    current_node = NextNode(); // Returned value by NextNode is a pointer to the next node.
    if(current_node->distance(*end_node) == 0)
    {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);   
  }
    return;
}