#ifndef ALTERNATIVE_PATH_ROUTING_HPP
#define ALTERNATIVE_PATH_ROUTING_HPP

#include "engine/datafacade.hpp"
#include "engine/internal_route_result.hpp"

#include "engine/algorithm.hpp"
#include "engine/search_engine_data.hpp"

#include "util/exception.hpp"

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

InternalManyRoutesResult alternativePathSearch(SearchEngineData<ch::Algorithm> &search_engine_data,
                                               const DataFacade<ch::Algorithm> &facade,
                                               const PhantomNodes &phantom_node_pair,
                                               unsigned number_of_alternatives);

InternalManyRoutesResult alternativePathSearch(SearchEngineData<mld::Algorithm> &search_engine_data,
                                               const DataFacade<mld::Algorithm> &facade,
                                               const PhantomNodes &phantom_node_pair,
                                               unsigned number_of_alternatives);

inline InternalManyRoutesResult alternativePathSearch(SearchEngineData<astar::Algorithm> &,
                                                      const DataFacade<astar::Algorithm> &,
                                                      const PhantomNodes &,
                                                      unsigned)
{
    throw util::exception("Not implemented");
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif
