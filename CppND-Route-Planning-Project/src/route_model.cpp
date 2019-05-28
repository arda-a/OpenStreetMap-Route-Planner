#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte>& xml) : Model(xml) {
  int counter = 0;
  for (Model::Node node : this->Nodes()) {
    this->m_nodes.push_back(RouteModel::Node(counter++, this, node));
  }

  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (auto& road : Roads()) {
    if (road.type != Road::Type::Footway) {
      for (int node_idx : Ways()[road.way].nodes) {
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = std::vector<const Road*>{};
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

void RouteModel::Node::FindNeighbors() {
  for (auto &road : parent_model->node_to_road[this->index]) {
    RouteModel::Node* node = this->FindNeighbor(this->parent_model->Ways()[road->way].nodes);
    if (node) {
      this->neighbors.push_back(node);
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
  Node* closest_node = nullptr;

  for (int i : node_indices) {
    Node* node = &parent_model->SNodes()[i];
    if (node->visited == false && this->distance(*node) != 0) {
      if (closest_node == nullptr || this->distance(*node) < this->distance(*closest_node)) {
        closest_node = node;
      }
    }
  }
  return closest_node;
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y)
{
  Node node;
  node.x = x;
  node.y = y;

  float min_dist = std::numeric_limits<float>::max();
  float dist = 0;
  int closest_idx = -1;
  for (const Model::Road& road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int idx : Ways()[road.way].nodes) {
        dist = SNodes()[idx].distance(node);
        if (dist < min_dist) {
          closest_idx = idx;
          min_dist = dist;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}