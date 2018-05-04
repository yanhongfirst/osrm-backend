#ifndef OSRM_ENGINE_ROUTING_BASE_ASTAR_HPP
#define OSRM_ENGINE_ROUTING_BASE_ASTAR_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <tuple>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{
namespace astar
{

// Heaps only record for each node its predecessor ("parent") on the shortest path.
// For re-constructing the actual path we need to trace back all parent "pointers".

using PackedPath = std::vector<NodeID>;

inline util::Coordinate getEntryCoordinate(const DataFacade<Algorithm> &facade, const NodeID node)
{
    auto index = facade.GetGeometryIndex(node);
    if (index.forward)
    {
        return facade.GetCoordinateOfNode(facade.GetUncompressedForwardGeometry(index.id).front());
    }
    else
    {
        return facade.GetCoordinateOfNode(facade.GetUncompressedReverseGeometry(index.id).front());
    }
}

inline EdgeWeight
lowerBoundToNode(const DataFacade<Algorithm> &facade, const NodeID from, const NodeID to)
{
    auto from_coordinate = getEntryCoordinate(facade, from);
    auto to_coordinate = getEntryCoordinate(facade, to);

    auto distance =
        util::coordinate_calculation::fccApproximateDistance(from_coordinate, to_coordinate);

    constexpr double MAX_SPEED = 140 / 3.6;

    return static_cast<EdgeWeight>(distance / MAX_SPEED * facade.GetWeightPrecision());
}

template <bool DIRECTION>
EdgeWeight
getNodePotential(const DataFacade<Algorithm> &facade, const NodeID node, const PhantomNodes &nodes)
{
    EdgeWeight forward_potential = INVALID_EDGE_WEIGHT;
    EdgeWeight reverse_potential = INVALID_EDGE_WEIGHT;

    if (nodes.target_phantom.forward_segment_id.enabled)
    {
        forward_potential =
            std::min(forward_potential,
                     lowerBoundToNode(facade, node, nodes.target_phantom.forward_segment_id.id) +
                         nodes.target_phantom.GetForwardWeightPlusOffset());
    }

    if (nodes.target_phantom.reverse_segment_id.enabled)
    {
        forward_potential =
            std::min(forward_potential,
                     lowerBoundToNode(facade, node, nodes.target_phantom.reverse_segment_id.id) +
                         nodes.target_phantom.GetReverseWeightPlusOffset());
    }

    if (nodes.source_phantom.forward_segment_id.enabled)
    {
        reverse_potential =
            std::min(reverse_potential,
                     lowerBoundToNode(facade, node, nodes.source_phantom.forward_segment_id.id) +
                         nodes.source_phantom.GetForwardWeightPlusOffset());
    }

    if (nodes.source_phantom.reverse_segment_id.enabled)
    {
        reverse_potential =
            std::min(reverse_potential,
                     lowerBoundToNode(facade, node, nodes.source_phantom.reverse_segment_id.id) +
                         nodes.source_phantom.GetReverseWeightPlusOffset());
    }

    if (DIRECTION == FORWARD_DIRECTION)
    {
        return (forward_potential - reverse_potential) / 2;
    }
    else
    {
        return (reverse_potential - forward_potential) / 2;
    }
}

template <bool DIRECTION, typename OutIter>
inline void retrievePackedPathFromSingleHeap(const SearchEngineData<Algorithm>::QueryHeap &heap,
                                             const NodeID middle,
                                             OutIter out)
{
    NodeID current = middle;
    NodeID parent = heap.GetData(current).parent;

    while (current != parent)
    {
        *out++ = parent;
        current = parent;
        parent = heap.GetData(parent).parent;
    }
}

template <bool DIRECTION>
inline PackedPath
retrievePackedPathFromSingleHeap(const SearchEngineData<Algorithm>::QueryHeap &heap,
                                 const NodeID middle)
{
    PackedPath packed_path;
    retrievePackedPathFromSingleHeap<DIRECTION>(heap, middle, std::back_inserter(packed_path));
    return packed_path;
}

// Trace path from middle to start in the forward search space (in reverse)
// and from middle to end in the reverse search space. Middle connects paths.

inline PackedPath
retrievePackedPathFromHeap(const SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                           const SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                           const NodeID middle)
{
    // Retrieve start -> middle. Is in reverse order since tracing back starts from middle.
    auto packed_path = retrievePackedPathFromSingleHeap<FORWARD_DIRECTION>(forward_heap, middle);
    std::reverse(begin(packed_path), end(packed_path));

    packed_path.emplace_back(middle);

    // Retrieve middle -> end. Is already in correct order, tracing starts from middle.
    auto into = std::back_inserter(packed_path);
    retrievePackedPathFromSingleHeap<REVERSE_DIRECTION>(reverse_heap, middle, into);

    return packed_path;
}

template <bool DIRECTION, typename Algorithm, typename... Args>
void relaxOutgoingEdges(const DataFacade<Algorithm> &facade,
                        typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                        const NodeID node,
                        const EdgeWeight weight,
                        const PhantomNodes &nodes)
{
    for (const auto edge : facade.GetAdjacentEdgeRange(node))
    {
        const auto &edge_data = facade.GetEdgeData(edge);

        if ((DIRECTION == FORWARD_DIRECTION) ? facade.IsForwardEdge(edge)
                                             : facade.IsBackwardEdge(edge))
        {
            const NodeID to = facade.GetTarget(edge);

            if (!facade.ExcludeNode(to))
            {
                const auto node_weight =
                    facade.GetNodeWeight(DIRECTION == FORWARD_DIRECTION ? node : to);
                const auto turn_penalty = facade.GetWeightPenaltyForEdgeID(edge_data.turn_id);

                const EdgeWeight to_weight = weight + node_weight + turn_penalty;
                const EdgeWeight potential = getNodePotential<DIRECTION>(facade, to, nodes);
                const EdgeWeight weight_estimate = to_weight + potential;

                if (!forward_heap.WasInserted(to))
                {
                    forward_heap.Insert(to, weight_estimate, {node});
                }
                else if (weight_estimate < forward_heap.GetKey(to))
                {
                    forward_heap.GetData(to) = {node};
                    forward_heap.DecreaseKey(to, weight_estimate);
                }
            }
        }
    }
}

template <bool DIRECTION, typename Algorithm, typename... Args>
void routingStep(const DataFacade<Algorithm> &facade,
                 typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                 typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                 NodeID &middle_node,
                 EdgeWeight &path_upper_bound,
                 const bool force_loop_forward,
                 const bool force_loop_reverse,
                 const PhantomNodes &nodes)
{
    const auto node = forward_heap.DeleteMin();
    const auto estimate = forward_heap.GetKey(node);
    const auto potential = getNodePotential<DIRECTION>(facade, node, nodes);
    const auto weight = estimate - potential;

    BOOST_ASSERT(!facade.ExcludeNode(node));

    // Upper bound for the path source -> target with
    // weight(source -> node) = weight weight(to -> target) ≤ reverse_weight
    // is weight + reverse_weight
    // More tighter upper bound requires additional condition reverse_heap.WasRemoved(to)
    // with weight(to -> target) = reverse_weight and all weights ≥ 0
    if (reverse_heap.WasInserted(node))
    {
        auto reverse_estimate = reverse_heap.GetKey(node);
        auto reverse_potential = getNodePotential<!DIRECTION>(facade, node, nodes);
        auto path_weight = weight + reverse_estimate - reverse_potential;

        // MLD uses loops forcing only to prune single node paths in forward and/or
        // backward direction (there is no need to force loops in MLD but in CH)
        if (!(force_loop_forward && forward_heap.GetData(node).parent == node) &&
            !(force_loop_reverse && reverse_heap.GetData(node).parent == node) &&
            (path_weight >= 0) && (path_weight < path_upper_bound))
        {
            middle_node = node;
            path_upper_bound = path_weight;
        }
    }

    // Relax outgoing edges from node
    relaxOutgoingEdges<DIRECTION>(facade, forward_heap, node, weight, nodes);
}

// With (s, middle, t) we trace back the paths middle -> s and middle -> t.
// This gives us a packed path (node ids) from the base graph around s and t,
// and overlay node ids otherwise. We then have to unpack the overlay clique
// edges by recursively descending unpacking the path down to the base graph.

using UnpackedNodes = std::vector<NodeID>;
using UnpackedEdges = std::vector<EdgeID>;
using UnpackedPath = std::tuple<EdgeWeight, UnpackedNodes, UnpackedEdges>;

template <typename Algorithm>
UnpackedPath search(SearchEngineData<Algorithm> &,
                    const DataFacade<Algorithm> &facade,
                    typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                    typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                    const bool force_loop_forward,
                    const bool force_loop_reverse,
                    EdgeWeight weight_upper_bound,
                    const PhantomNodes &phantom_nodes)
{
    if (forward_heap.Empty() || reverse_heap.Empty())
    {
        return std::make_tuple(INVALID_EDGE_WEIGHT, std::vector<NodeID>(), std::vector<EdgeID>());
    }

    BOOST_ASSERT(!forward_heap.Empty() && forward_heap.MinKey() < INVALID_EDGE_WEIGHT);
    BOOST_ASSERT(!reverse_heap.Empty() && reverse_heap.MinKey() < INVALID_EDGE_WEIGHT);

    // run two-Target Dijkstra routing step.
    NodeID middle = SPECIAL_NODEID;
    EdgeWeight weight = weight_upper_bound;
    EdgeWeight forward_heap_min = forward_heap.MinKey();
    EdgeWeight reverse_heap_min = reverse_heap.MinKey();
    while (forward_heap.Size() + reverse_heap.Size() > 0 &&
           forward_heap_min + reverse_heap_min < weight)
    {
        if (!forward_heap.Empty())
        {
            routingStep<FORWARD_DIRECTION>(facade,
                                           forward_heap,
                                           reverse_heap,
                                           middle,
                                           weight,
                                           force_loop_forward,
                                           force_loop_reverse,
                                           phantom_nodes);
            if (!forward_heap.Empty())
                forward_heap_min = forward_heap.MinKey();
        }
        if (!reverse_heap.Empty())
        {
            routingStep<REVERSE_DIRECTION>(facade,
                                           reverse_heap,
                                           forward_heap,
                                           middle,
                                           weight,
                                           force_loop_reverse,
                                           force_loop_forward,
                                           phantom_nodes);
            if (!reverse_heap.Empty())
                reverse_heap_min = reverse_heap.MinKey();
        }
    };

    // No path found for both target nodes?
    if (weight >= weight_upper_bound || SPECIAL_NODEID == middle)
    {
        return std::make_tuple(INVALID_EDGE_WEIGHT, std::vector<NodeID>(), std::vector<EdgeID>());
    }

    // Get (un)packed path as edges {from node ID, to node ID, from_clique_arc}
    auto unpacked_nodes = retrievePackedPathFromHeap(forward_heap, reverse_heap, middle);
    std::vector<EdgeID> unpacked_edges;
    unpacked_edges.reserve(unpacked_nodes.size());

    // Beware the edge case when start, middle, end are all the same.
    // In this case we return a single node, no edges. We also don't unpack.
    if (!unpacked_nodes.empty())
    {
        for (auto iter = unpacked_nodes.begin(); iter != std::prev(unpacked_nodes.end()); ++iter)
        {
            unpacked_edges.push_back(facade.FindEdge(*iter, *std::next(iter)));
        }
    }

    return std::make_tuple(weight, std::move(unpacked_nodes), std::move(unpacked_edges));
}

// Alias to be compatible with the CH-based search
template <typename Algorithm>
inline void search(SearchEngineData<Algorithm> &search_engine_data,
                   const DataFacade<Algorithm> &facade,
                   typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                   typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                   EdgeWeight &weight,
                   std::vector<NodeID> &unpacked_nodes,
                   const bool force_loop_forward,
                   const bool force_loop_reverse,
                   const PhantomNodes &phantom_nodes,
                   const EdgeWeight weight_upper_bound = INVALID_EDGE_WEIGHT)
{
    // TODO: change search calling interface to use unpacked_edges result
    std::tie(weight, unpacked_nodes, std::ignore) = search(search_engine_data,
                                                           facade,
                                                           forward_heap,
                                                           reverse_heap,
                                                           force_loop_forward,
                                                           force_loop_reverse,
                                                           weight_upper_bound,
                                                           phantom_nodes);
}

// TODO: refactor CH-related stub to use unpacked_edges
template <typename RandomIter, typename FacadeT>
void unpackPath(const FacadeT &facade,
                RandomIter packed_path_begin,
                RandomIter packed_path_end,
                const PhantomNodes &phantom_nodes,
                std::vector<PathData> &unpacked_path)
{
    const auto nodes_number = std::distance(packed_path_begin, packed_path_end);
    BOOST_ASSERT(nodes_number > 0);

    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    unpacked_nodes.reserve(nodes_number);
    unpacked_edges.reserve(nodes_number);

    unpacked_nodes.push_back(*packed_path_begin);
    if (nodes_number > 1)
    {
        util::for_each_pair(
            packed_path_begin,
            packed_path_end,
            [&facade, &unpacked_nodes, &unpacked_edges](const auto from, const auto to) {
                unpacked_nodes.push_back(to);
                unpacked_edges.push_back(facade.FindEdge(from, to));
            });
    }

    annotatePath(facade, phantom_nodes, unpacked_nodes, unpacked_edges, unpacked_path);
}

template <typename Algorithm>
double getNetworkDistance(SearchEngineData<Algorithm> &search_engine_data,
                          const DataFacade<Algorithm> &facade,
                          typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                          typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                          const PhantomNode &source_phantom,
                          const PhantomNode &target_phantom,
                          EdgeWeight weight_upper_bound = INVALID_EDGE_WEIGHT)
{
    forward_heap.Clear();
    reverse_heap.Clear();

    const PhantomNodes phantom_nodes{source_phantom, target_phantom};
    insertNodesInHeaps(forward_heap, reverse_heap, phantom_nodes);

    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    std::tie(weight, unpacked_nodes, unpacked_edges) = search(search_engine_data,
                                                              facade,
                                                              forward_heap,
                                                              reverse_heap,
                                                              DO_NOT_FORCE_LOOPS,
                                                              DO_NOT_FORCE_LOOPS,
                                                              weight_upper_bound,
                                                              phantom_nodes);

    if (weight == INVALID_EDGE_WEIGHT)
    {
        return std::numeric_limits<double>::max();
    }

    std::vector<PathData> unpacked_path;

    annotatePath(facade, phantom_nodes, unpacked_nodes, unpacked_edges, unpacked_path);

    return getPathDistance(facade, unpacked_path, source_phantom, target_phantom);
}

} // namespace astar
} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif // OSRM_ENGINE_ROUTING_BASE_MLD_HPP
