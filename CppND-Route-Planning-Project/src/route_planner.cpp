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

float RoutePlanner::CalculateHValue(const RouteModel::Node* node)
{
  return this->end_node->distance(*node);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node)
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
  std::sort(open_list.begin(), open_list.end(), [](const auto& _first, const auto& _second) {
    return _first->h_value + _first->g_value < _second->h_value + _second->g_value;
    });

  RouteModel::Node* ptr_lowest_f = *open_list.begin();
  open_list.erase(open_list.begin());
  return ptr_lowest_f;
}

void RoutePlanner::AStarSearch()
{
  end_node->parent = start_node;
  m_Model.path = ConstructFinalPath(end_node);
}