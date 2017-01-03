#include "world.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

World::World(const Config& config)
{
    // get the world markers from the config
    for (const auto& marker : config.markers())
    {
        if (marker.ref == "world")
        {
            m_worldMarkers[marker.id].rot = marker.transf.getRotation();
            m_worldMarkers[marker.id].pos = marker.transf.getOrigin();
        }
    }
}

void World::update(const FilteredSensorDataMap& sensorData)
{
    // check if an entity knows the pose of a world marker
    // if that's the case, it is possible to calculate the entitiy's
    // pose in world coordinates
    //TODO: implement

    //now that we have the world pose of some (or all) entities, we can use
    // those to calculate the pose of the other entities

    // build a (dependency) graph
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS>;

    Graph graph;
    graph.added_vertex(0);
    graph.added_vertex(1);

    boost::add_edge(0, 1, graph);
}
