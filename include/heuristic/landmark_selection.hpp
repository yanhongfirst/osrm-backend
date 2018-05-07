#ifndef OSRM_HEURISTIC_LANDMARK_SELECTION_HPP
#define OSRM_HEURISTIC_LANDMARK_SELECTION_HPP

#include "util/integer_range.hpp"
#include "util/query_heap.hpp"
#include "util/typedefs.hpp"

#include <boost/function_output_iterator.hpp>

#include <random>
#include <set>
#include <vector>

namespace osrm
{
namespace heuristic
{
namespace detail
{
struct HeapData
{
    NodeID parent;
};
struct Edge
{
    NodeID from;
    NodeID to;

    bool operator<(const Edge &other) const
    {
        return std::tie(from, to) < std::tie(other.from, other.to);
    }

    bool operator==(const Edge &other) const
    {
        return std::tie(from, to) == std::tie(other.from, other.to);
    }
};
using EdgeList = std::vector<Edge>;

inline NodeID findRoot(const EdgeList &edge_list, const std::size_t number_of_nodes)
{
    BOOST_ASSERT(std::is_sorted(edge_list.begin(), edge_list.end()));

    std::vector<bool> has_parent(number_of_nodes, false);
    for (const auto &edge : edge_list)
    {
        has_parent[edge.to] = true;
    }

    auto iter = std::find(has_parent.begin(), has_parent.end(), false);
    BOOST_ASSERT_MSG(iter != has_parent.end(), "Given edge list is not a tree");

    return *iter;
}

struct SearchTree
{

    SearchTree(EdgeList edge_list_, const std::size_t number_of_nodes)
        : edge_list(std::move(edge_list_)),
          number_of_nodes{number_of_nodes}, root{findRoot(edge_list, number_of_nodes)}
    {
        std::sort(edge_list.begin(), edge_list.end());
    }

    template <typename OutIter> OutIter GetChildren(const NodeID node, OutIter out) const
    {
        auto iter = std::lower_bound(
            edge_list.begin(), edge_list.end(), node, [](const auto &edge, const NodeID node) {
                return edge.from < node;
            });
        while (iter != edge_list.end() && iter->from == node)
        {
            *out++ = iter->to;
            iter++;
        }
        return out;
    }

    template <typename Func> void DepthFirstVisit(Func &&func) const
    {
        std::vector<NodeID> nodes;
        nodes.reserve(number_of_nodes);

        nodes.push_back(root);
        int index = 0;
        while (index < static_cast<int>(nodes.size()))
        {
            GetChildren(nodes[index], std::back_inserter(nodes));
            index++;
        }
        BOOST_ASSERT(index == nodes.size());

        while (index > 0)
        {
            index--;
            func(nodes[index]);
        }
    }

    template <typename Comp> NodeID LargestLeaf(const NodeID root, Comp &&comp) const
    {
        std::vector<NodeID> children;

        NodeID node = root;
        GetChildren(node, std::back_inserter(children));
        while (children.size() > 0)
        {
            auto max_child = *std::max_element(children.begin(), children.end(), comp);
            node = max_child;
            children.clear();
            GetChildren(node, std::back_inserter(children));
        }

        return node;
    }

    std::vector<Edge> edge_list;
    const std::size_t number_of_nodes;
    const NodeID root;
};

using Heap = util::QueryHeap<NodeID, NodeID, EdgeWeight, HeapData, util::ArrayStorage<NodeID, int>>;

template <typename GraphT, typename HeapT, bool DIRECTION>
void oneToAll(const GraphT &graph, const HeapT &heap, const NodeID start)
{
    heap.Insert(start, {start});

    while (!heap.Empty())
    {
        const auto node = heap.DeleteMin();
        const auto weight = heap.GetKey(node);
        for (const auto edge : graph.GetAdjacentEdgeRange(node))
        {
            auto to = graph.GetTarget(edge);
            const auto &data = graph.GetEdgeData(edge);
            if (data.forward == DIRECTION)
            {
                auto edge_weight = graph.GetEdgeData(edge).weight;
                auto total_weight = weight + edge_weight;

                if (!heap.WasInserted(to))
                {
                    heap.Insert(to, {node});
                }
                else if (total_weight < heap.GetKey(to))
                {
                    heap.DecreaseKey(to, total_weight);
                    heap.GetData(to).parent = node;
                }
            }
        }
    }
}

template <typename HeapT, typename GraphT>
std::vector<EdgeWeight> extractWeights(const GraphT &graph, const HeapT &heap)
{
    std::vector<EdgeWeight> weights(graph.GetNumberOfNodes(), INVALID_EDGE_WEIGHT);

    for (auto node : util::irange<NodeID>(0, graph.GetNumberOfNodes()))
    {
        weights[node] = heap.GetKey(node);
    }

    return weights;
}

template <typename HeapT, typename GraphT>
EdgeList extractSearchTree(const GraphT &graph, const HeapT &heap)
{
    EdgeList edge_list;

    for (auto node : util::irange<NodeID>(0, graph.GetNumberOfNodes()))
    {
        auto parent = heap.GetData(node).parent;
        edge_list.emplace_back(parent, node);
    }

    return edge_list;
}

EdgeWeight getALTPotential(const NodeID node,
                           const NodeID target,
                           const std::vector<std::vector<EdgeWeight>> &to_landmark,
                           const std::vector<std::vector<EdgeWeight>> &from_landmark)
{
    EdgeWeight potential = INVALID_EDGE_WEIGHT;

    for (auto index : util::irange<std::size_t>(0, to_landmark.size()))
    {
        auto positive = from_landmark[index][target] - from_landmark[index][node];
        auto negative = to_landmark[index][node] - to_landmark[index][target];
        potential = std::max(potential, std::max(positive, negative));
    }

    return potential;
}

} // namespace detail

// Implements the Avoid landmark selection strategy
template <typename GraphT>
std::vector<NodeID> selectLandmarks(const GraphT &graph, const std::size_t number_of_landmarks)
{
    std::set<NodeID> landmarks;
    std::vector<std::vector<EdgeWeight>> to_landmark;
    std::vector<std::vector<EdgeWeight>> from_landmark;
    detail::Heap heap(graph.GetNumberOfNodes());

    const constexpr bool FORWARD_DIRECTION = true;
    const constexpr bool REVERSE_DIRECTION = false;

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, graph.GetNumberOfNodes() - 1);

    std::vector<EdgeWeight> subtree_size(graph.GetNumberOfNodes());
    std::vector<bool> subtree_has_landmark(graph.GetNumberOfNodes());
    for (auto index : util::irange<std::size_t>(0, number_of_landmarks))
    {
        (void) index;
        NodeID root = dist(rng);

        heap.Clear();
        detail::oneToAll<FORWARD_DIRECTION>(graph, heap, root);
        auto from_root_weights = detail::extractWeights(graph, heap);

        auto edge_list = detail::extractSearchTree(graph, heap);
        detail::SearchTree shortest_path_tree{std::move(edge_list), graph.GetNumberOfNodes()};

        // identify all subtrees that have a landmark (perfectly covered)
        shortest_path_tree.DepthFirstVisit([&](const NodeID node) {
            subtree_has_landmark[node] = landmarks.count(node) > 0;
            shortest_path_tree.GetChildren(
                node, boost::make_function_output_iterator([&](const NodeID child) {
                    subtree_has_landmark[node] =
                        subtree_has_landmark[node] || subtree_has_landmark[child];
                }));
        });

        heap.Clear();
        detail::oneToAll<REVERSE_DIRECTION>(graph, heap, root);
        auto to_root_weights = detail::extractWeights(graph, heap);

        // compute size of all subtrees that are not covered
        shortest_path_tree.DepthFirstVisit([&](const NodeID node) {
            if (!subtree_has_landmark[node])
            {
                subtree_size[node] = 0;
            }
            else
            {
                const auto potential =
                    detail::getALTPotential(node, root, to_root_weights, from_root_weights);
                subtree_size[node] = from_root_weights[node] - potential;
                shortest_path_tree.GetChildren(
                    node, boost::make_function_output_iterator([&](const NodeID child) {
                        subtree_size[node] += subtree_size[child];
                    }));
            }
        });

        auto max_iter = std::max_element(subtree_size.begin(), subtree_size.end());
        auto worst_subtree_root = std::distance(subtree_size.begin(), max_iter);

        auto landmark = shortest_path_tree.LargestLeaf(worst_subtree_root, [&](const auto lhs, const auto rhs) {
            return subtree_size[lhs] < subtree_size[rhs];
        });

        landmarks.emplace(landmark);

        heap.Clear();
        detail::oneToAll<FORWARD_DIRECTION>(graph, heap, root);
        auto from_landmark_weights = detail::extractWeights(graph, heap);
        from_landmark.emplace_back(std::move(from_landmark_weights));

        heap.Clear();
        detail::oneToAll<REVERSE_DIRECTION>(graph, heap, root);
        auto to_landmark_weights = detail::extractWeights(graph, heap);
        to_landmark.emplace_back(std::move(to_landmark_weights));
    }
}

// Implements the maxCover landmark selection strategy
template <typename GraphT>
std::vector<NodeID> reduceLandsmarks(const GraphT &graph, const std::size_t number_of_landmarks)
{
}


} // namespace heuristic
} // namespace osrm

#endif
