#ifndef OSRM_HEURISTIC_DISTANCE_POTENTIAL_HPP
#define OSRM_HEURISTIC_DISTANCE_POTENTIAL_HPP

#include "engine/datafacade.hpp"

#include "util/typedefs.hpp"

namespace osrm
{
namespace heuristic
{
template <typename Algorithm> class DistancePotential
{
  public:
    DistancePotential(const engine::DataFacade<Algorithm> &facade,
                      const engine::PhantomNode &target)
        : facade(facade), use_forward(target.forward_segment_id.enabled),
          use_reverse(target.reverse_segment_id.enabled)

    {
        if (use_forward)
        {
            forward_target = GetEntryCoordinate(facade, target.forward_segment_id.id);
        }
        if (use_reverse)
        {
            reverse_target = GetEntryCoordinate(facade, target.reverse_segment_id.id);
        }
    }

    EdgeWeight operator[](const NodeID node) const
    {
        EdgeWeight potential = INVALID_EDGE_WEIGHT;

        if (use_forward)
        {
            potential = LowerBoundToNode(node, forward_target);
        }
        if (use_reverse)
        {
            potential = std::min(potential, LowerBoundToNode(node, reverse_target));
        }

        return potential;
    }

  private:
    EdgeWeight LowerBoundToNode(const NodeID from, const util::Coordinate to_coordinate) const
    {
        auto from_coordinate = GetEntryCoordinate(facade, from);

        auto distance =
            util::coordinate_calculation::fccApproximateDistance(from_coordinate, to_coordinate);

        constexpr double MAX_SPEED = 140 / 3.6;

        return static_cast<EdgeWeight>(distance / MAX_SPEED * facade.GetWeightPrecision());
    }

    util::Coordinate GetEntryCoordinate(const engine::DataFacade<Algorithm> &facade,
                                        const NodeID node) const
    {
        auto index = facade.GetGeometryIndex(node);
        if (index.forward)
        {
            return facade.GetCoordinateOfNode(
                facade.GetUncompressedForwardGeometry(index.id).front());
        }
        else
        {
            return facade.GetCoordinateOfNode(
                facade.GetUncompressedReverseGeometry(index.id).front());
        }
    }

    const engine::DataFacade<Algorithm> &facade;
    util::Coordinate forward_target;
    util::Coordinate reverse_target;
    bool use_forward;
    bool use_reverse;
};

} // namespace heuristic
} // namespace osrm

#endif
