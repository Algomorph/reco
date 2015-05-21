#ifndef BOOST_MST_KRUSKAL_HPP
#define BOOST_MST_KRUSKAL_HPP

// Includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>

#include <adjacency_list.hpp>

// Typedefs
typedef boost::adjacency_list < boost::vecS, 
                                boost::vecS,
                                boost::undirectedS,
                                boost::no_property,
                                boost::property < boost::edge_weight_t, float > > Graph;
                                
typedef boost::graph_traits < Graph >::edge_descriptor Edge;
typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;
typedef std::pair<int, int> E;
                                    
////////////////////////////////////////////////////////////////////////////////

/** \brief Compute the minimum spaning tree of the graph using Kruskal's algorithm
  * \param[in]  edges         edges of the graph.
  * \param[in]  edge_weights  weights of the edges
  * \param[out] msts          MST for each connected component of the original graph
  * \note http://www.boost.org/doc/libs/1_57_0/libs/graph/doc/kruskal_min_spanning_tree.html
  * \note Inpput graph:
  * \note   * can have multiple connected components
  * \note   * vertex indices do not have to be continuous
  */
bool boostMSTKruskal(  const std::vector<std::vector<int> >         &edges, 
                       const std::vector<std::vector<float> >       &edge_weights,
                       std::vector<std::vector<std::vector<int> > > &msts
                    )
{
  //////////////////////////////////////////////////////////////////////////////
  // Check that edges and weights have same dimensions
  
  if (edges.size() != edge_weights.size())
  {
    std::cout << "[boostMSTKruskal] number of input edges and edge weights are not equal.\n";
    return false;
  }
  
  for (size_t sourceId = 0; sourceId < edges.size(); sourceId++)
  {
    if (edges[sourceId].size() != edge_weights[sourceId].size())
    {
      std::cout << "[boostMSTKruskal] number of input edges and edge weights are not equal.\n";
      return false;
    }      
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Convert edges
  
  std::vector<E> edges_pair;
  std::vector<float> edge_weights_pair;
  for (size_t sourceId = 0; sourceId < edges.size(); sourceId++)
  {
    for (size_t targetIdItr = 0; targetIdItr < edges[sourceId].size(); targetIdItr++)
    {
      int targetId = edges[sourceId][targetIdItr];
      
      if (targetId > sourceId)
        continue;
      
      edges_pair.push_back(E(sourceId, targetId));
      edge_weights_pair.push_back(edge_weights[sourceId][targetIdItr]);
    }
  }
    
  //////////////////////////////////////////////////////////////////////////////
  // Create graph
  
  int numVertices = edges.size();
  Graph g (&edges_pair[0], &edges_pair[0]+edges_pair.size(), &edge_weights_pair[0], numVertices);
    
  //////////////////////////////////////////////////////////////////////////////
  // Find MST

  std::vector < Edge > spanning_tree;
  kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
  
  //////////////////////////////////////////////////////////////////////////////
  // Find connected components of the graph
  
  // Find connected components
  std::vector<int> CC(numVertices);
  int numCCs = boost::connected_components(g, &CC[0]);
  
  // Remove connected components containing a single vertex
  std::vector<int> CCRemap (numCCs, -1);
  int numValidCC = 0;
  for (size_t i = 0; i < numCCs; i++)
  {
    int CCSize = std::count(CC.begin(), CC.end(), static_cast<int>(i));
    
    if (CCSize > 1)
      CCRemap[i] = numValidCC++;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // Prepare output

  msts.clear();
  msts.resize(numValidCC);
  for (size_t CC = 0; CC < numValidCC; CC++)
    msts[CC].resize(numVertices);
  
  // Go through all edges of the MST and add them to the corresponding connected component  
  for (size_t i = 0; i < spanning_tree.size(); i++)
  {
    int v1    = boost::source(spanning_tree[i], g);
    int v2    = boost::target(spanning_tree[i], g);
    int CCid  = CCRemap[CC[v1]];
    
    utl::addEdge(v1, v2, msts[CCid]);
  }
    
  //////////////////////////////////////////////////////////////////////////////
  // Debug
  
  // Print the MST
//   boost::property_map < Graph, boost::edge_weight_t >::type weight = get(boost::edge_weight, g);    
//   std::cout << "Print the edges in the MST:" << std::endl;
//   for (std::vector < Edge >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei)
//   {
//     std::cout << boost::source(*ei, g) << " <--> " << boost::target(*ei, g) << " with weight of " << weight[*ei] << std::endl;
//   }

//   // Print connected component remap
//   for (size_t i = 0; i < numCCs; i++)
//     std::cout << "Connected component " << i << " -> " << CCRemap[i] << std::endl;

  
//   // Print connected components of the graph
//   std::vector<int>::size_type i;
//   std::cout << "Total number of components: " << numCCs << std::endl;
//   for (i = 0; i != CC.size(); ++i)
//   {
//     std::cout << "Vertex " << i <<" is in component " << CC[i] << std::endl;
//   }
//   std::cout << std::endl;  
//     
//   // Print MSTs
//   for (size_t i = 0; i < msts.size(); i++)
//   {
//     std::cout << "MST " << i << ":\n";
//     for (size_t j = 0; j < msts[i].size(); j++)
//       std::cout<< "   " << msts[i][j].first << ", " << msts[i][j].second << std::endl;
//   }
  
  return true;
}

# endif // BOOST_MST_KRUSKAL_HPP