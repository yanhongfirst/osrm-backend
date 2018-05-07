#ifndef OSRM_HEURISTIC_ALT_POTENTIAL_HPP
#define OSRM_HEURISTIC_ALT_POTENTIAL_HPP

#include "engine/phantom_node.hpp"

#include "util/exception.hpp"
#include "util/integer_range.hpp"
#include "util/typedefs.hpp"

#include <array>
#include <vector>

namespace osrm
{
namespace heuristic
{

template <unsigned NUM_LANDMARKS> struct ALTPotential;

template <unsigned NUM_LANDMARKS> class ALTPoentialFactory
{
    using DistanceValues = std::array<EdgeWeight, NUM_LANDMARKS>;

  public:
    ALTPoentialFactory(const std::vector<std::vector<EdgeWeight>> &to_landmark_,
                       const std::vector<std::vector<EdgeWeight>> &from_landmark_)
    {
        auto number_of_nodes = to_landmark_.front().size();
        BOOST_ASSERT(from_landmark_.front().size() == number_of_landmarks);

        auto number_of_landmarks = to_landmark_.size();
        if (number_of_landmarks != NUM_LANDMARKS)
        {
            throw util::exception("Number of landmarks is not " + std::to_string(NUM_LANDMARKS));
        }

        to_landmark.resize(number_of_nodes);
        from_landmark.resize(number_of_nodes);

        // pack values for cache efficency
        for (auto index : util::irange<std::size_t>(0, NUM_LANDMARKS))
        {
            for (auto node : util::irange<NodeID>(0, number_of_nodes))
            {
                from_landmark[node][index] = from_landmark_[index][node];
                to_landmark[node][index] = to_landmark_[index][node];
            }
        }
    }

    ALTPotential<NUM_LANDMARKS> MakePotential(const engine::PhantomNode &target) const
    {
        ALTPotential<NUM_LANDMARKS>{target, to_landmark, from_landmark};
    }

  private:
    std::vector<DistanceValues> to_landmark;
    std::vector<DistanceValues> from_landmark;
};

template <unsigned NUM_LANDMARKS> struct ALTPotential
{
    using DistanceValues = std::array<EdgeWeight, NUM_LANDMARKS>;

  public:
    ALTPotential(const engine::PhantomNode &target,
                 const std::vector<DistanceValues> &to_landmark,
                 const std::vector<DistanceValues> &from_landmark)
        : use_forward(target.forward_segment_id.enabled),
          use_reverse(target.reverse_segment_id.enabled), target(target), to_landmark(to_landmark),
          from_landmark(from_landmark)
    {
    }

    EdgeWeight operator[](const NodeID node) const
    {
        EdgeWeight potential = INVALID_EDGE_WEIGHT;
        if (use_forward)
        {
            potential = GetPotential(node, target.forward_segment_id.id);
        }
        if (use_reverse)
        {
            potential = std::min(potential, GetPotential(node, target.reverse_segment_id.id));
        }
        return potential;
    }

  private:
    EdgeWeight GetPotential(const NodeID node, const NodeID target) const
    {
        auto positive = from_landmark[target] - from_landmark[node];
        auto negative = to_landmark[node] - to_landmark[target];
        auto max_positive = std::max_element(positive.begin(), positive.end());
        auto max_negative = std::max_element(negative.begin(), negative.end());
        return std::max(*max_positive, *max_negative);
    }

    bool use_forward;
    bool use_reverse;
    const engine::PhantomNode &target;
    const std::vector<DistanceValues> &to_landmark;
    const std::vector<DistanceValues> &from_landmark;
};

} // namespace heuristic
} // namespace osrm

#endif
