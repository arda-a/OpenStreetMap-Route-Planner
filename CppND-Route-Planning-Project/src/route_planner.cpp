#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
  this->m_Model = model;

  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node)
{
  current_node->FindNeighbors();
  for (RouteModel::Node* neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    float f_val = current_node->distance(*neighbor);
    neighbor->g_value = current_node->g_value + f_val;
    neighbor->h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node * node)
{
  return this->end_node->distance(*node);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node * current_node)
{
  std::vector<RouteModel::Node> path_found;
  distance = 0.0f;

  while (current_node->parent != nullptr) {
    path_found.push_back(*current_node);
    auto parent = *(current_node->parent);
    distance += current_node->distance(parent);
    current_node = current_node->parent;
  }

  path_found.push_back(*current_node);
  distance *= m_Model.MetricScale();
  return path_found;
}

RouteModel::Node* RoutePlanner::NextNode()
{
  std::sort(open_list.begin(), open_list.end(), [](const auto & _first, const auto & _second) {
    return _first->h_value + _first->g_value < _second->h_value + _second->g_value;
    });

  RouteModel::Node* ptr_lowest_f = *open_list.begin();
  open_list.erase(open_list.begin());
  return ptr_lowest_f;
}

void RoutePlanner::AStarSearch()
{
  start_node->visited = true;
  open_list.push_back(start_node);
  RouteModel::Node* current_node = nullptr;

  while (open_list.size() > 0) {
    current_node = NextNode();
    if (current_node->distance(*end_node) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    else {
      AddNeighbors(current_node);
    }
  }
}