#ifndef OSRM_HEURISTIC_BIDIRECTIONAL_POTENTIAL_HPP
#define OSRM_HEURISTIC_BIDIRECTIONAL_POTENTIAL_HPP

#include "util/typedefs.hpp"

namespace osrm
{
namespace heuristic
{

template <typename Potential> struct BidirectionalPotential
{
    BidirectionalPotential(const Potential &forward_potential, const Potential &reverse_potential)
        : forward_potential(forward_potential), reverse_potential(reverse_potential)
    {
    }

    EdgeWeight Forward(const NodeID node) const
    {
        return (forward_potential[node] - reverse_potential[node]) / 2;
    }

    EdgeWeight Reverse(const NodeID node) const
    {
        return (reverse_potential[node] - forward_potential[node]) / 2;
    }

    const Potential &forward_potential;
    const Potential &reverse_potential;
};

template <typename Potential>
BidirectionalPotential<Potential> makePotential(const Potential &forward_potential,
                                                const Potential &reverse_potential)
{
    return {forward_potential, reverse_potential};
}

} // namespace heuristic
} // namespace osrm

#endif
