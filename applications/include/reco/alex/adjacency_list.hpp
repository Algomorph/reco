#ifndef ADJACENCY_LIST_HPP
#define ADJACENCY_LIST_HPP

#include <queue>

namespace utl
{
  /** Graph datastructure represented as a vector of vectors. The indices of the 
   * outer vector correspond to the indices of the vertices in the graph. The 
   * the inner graph correspond to the indices of the vertices that the outter 
   * indices of index vertex is connected to. To avoid duplicates a record of an 
   * edge between two vertices is stored only in the inner vector corresponding to
   * the vertex with a greater index.
   */  
  typedef std::vector<std::vector<int> > Graph;
  
  /** Add edge to the graph
   * \param[in] v1    index of first vertex
   * \param[in] v2    index of second vertex
   * \param[in] g     graph object
   * \return    false if any of the vertices is out of bounds, otherwise true
   */
  inline
  bool addEdge(const int &v1, const int &v2, Graph &g)
  {
    // Check that edge is in bounds
    if (v1 > g.size() || v2 > g.size() || v1 < 0 || v2 < 0)
    {
      std::cout << "[utl::addEdge] vertex indices are out of bounds." << std::endl;
      return false;
    }
    
    // Check that it is a valid edge
    if (v1 == v2)
    {
      std::cout << "[utl::addEdge] edge must be between two different vertices." << std::endl;
      return false;
    }
    
    // Add edge if it does not exist
    if (std::find(g[v1].begin(), g[v1].end(), v2) == g[v1].end())
      g[v1].push_back(v2);
    
    if (std::find(g[v2].begin(), g[v2].end(), v1) == g[v2].end())
      g[v2].push_back(v1);
    
    return true;
  }

  /** Get number of edges in the graph
   * \param[in] graph   input graph
   * \return    number of edges in the graph
   */
  inline
  int getNumEdges (const Graph &g)
  {
    int numEdges = 0;
    for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
      numEdges += g[vertexId].size();
    
    return numEdges;
  }
  
  /** Print the edges in the graph
   * \param[in] graph   input graph
   */
  inline
  void printGraph (const Graph &g)
  {
    for (size_t sourceVtx = 0; sourceVtx < g.size(); sourceVtx++)
    {
      for (size_t targetVtxId = 0; targetVtxId < g[sourceVtx].size(); targetVtxId++)
      {
        int targetVtx = g[sourceVtx][targetVtxId];
        std::cout << "   " << sourceVtx << " -> " << targetVtx << std::endl;
      }
    }    
  }  
  
  /** Convert graph to a vector of pairs representing edges
   * \param[in]       g         corresponding graph
   * \return vector of pairs where each pair represents an edge
   */
  inline
  std::vector<std::pair<int,int> > graph2EdgePairs (const Graph &g)
  {
    std::vector<std::pair<int,int> > edgePairs;
    
    for (size_t sourceId = 0; sourceId < g.size(); sourceId++)
    {
      for (size_t targetIdIt = 0; targetIdIt < g[sourceId].size(); targetIdIt++)
      {
        int targetId = g[sourceId][targetIdIt];
        if (targetId > sourceId)
          edgePairs.push_back(std::pair<int,int>(sourceId, targetId));
      }
    }
    
    return edgePairs;
  }

  /** Given a set of vertices in a graph find all adjacent vertices of the input
   * vertices and return edges connecting input edges to adjacent edges. Note 
   * that none of the input vertices can be in the set of adjacent vertices.
   * \param[in] g graph
   * \param[in] v input vertices
   * \return vector of pairs where each pair represents an edge
   */
  inline
  std::vector<std::pair<int,int> > getAdjacentVertexEdges (const Graph &g, const std::vector<int> &v)
  {
    // Find all edges between segment and it's neighbours
    std::vector<std::pair<int,int> > nighbour_edges;
    std::vector<int>::const_iterator v1It = v.begin();
    for ( ; v1It != v.end(); v1It++)
    {
      std::vector<int>::const_iterator v2It = g[*v1It].begin();
      for (; v2It != g[*v1It].end(); v2It++)
      {
        // Check if it is a neighbour segment
        if(std::find(v.begin(), v.end(), *v2It) == v.end())
        {
          nighbour_edges.push_back(std::pair<int,int> (*v1It, *v2It));
        }
      }
    }
    
    return nighbour_edges;
  }

  /** Given a set of vertices in a graph return all edges belonging to the cut 
   * between the input set of vertices and the rest of the vertices in the 
   * graph.
   * \param[in] g graph
   * \param[in] v input vertices
   * \return edges belonging to the cut
   */
  inline
  std::vector<std::pair<int,int> > getCutEdges (const Graph &g, const std::vector<int> &v)
  {
    // Find all edges between segment and it's neighbours
    std::vector<std::pair<int,int> > cut_edges;
    std::vector<int>::const_iterator v1It = v.begin();
    for ( ; v1It != v.end(); v1It++)
    {
      std::vector<int>::const_iterator v2It = g[*v1It].begin();
      for (; v2It != g[*v1It].end(); v2It++)
      {
        // Check if it is a neighbour segment
        if(std::find(v.begin(), v.end(), *v2It) == v.end())
        {
          cut_edges.push_back(std::pair<int,int> (*v1It, *v2It));
        }
      }
    }
    
    return cut_edges;
  }  
  
  /** Given a set of vertices in a graph return the edges of the subgraph formed
   * by these vertices.
   * \param[in] g graph
   * \param[in] v subgraph vertices
   * \return vector of pairs where each pair represents an edge in the subgraph
   */
  inline
  std::vector<std::pair<int,int> > getSubgraphEdges (const Graph &g, const std::vector<int> &v)
  {
    // Find all edges between segment and it's neighbours
    std::vector<std::pair<int,int> > subgraph_edges;
    std::vector<int>::const_iterator v1It = v.begin();
    for ( ; v1It != v.end(); v1It++)
    {
      std::vector<int>::const_iterator v2It = g[*v1It].begin();
      for (; v2It != g[*v1It].end(); v2It++)
      {
        // Check if it is a neighbour segment
        if(std::find(v.begin(), v.end(), *v2It) != v.end())
        {
          subgraph_edges.push_back(std::pair<int,int> (*v1It, *v2It));
        }
      }
    }
    
    return subgraph_edges;
  }  
  
  /** Find leaves in a graph
   * \param[in]   g input graph
   * \return indices of leaf vertices
   */
  inline
  std::vector<int> getGraphLeaves ( const Graph  &g)
  {    
    // Leaves are edges with a single outgoing edge
    std::vector<int> leaves;
    for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
    {
      if (g[vertexId].size() == 1)
        leaves.push_back(vertexId);
    }    
    return leaves;
  }  
  
  /** Find connected components in the graph
   * \param[in] g     graph object
   * \return    a vector of vectors where each inner vector corresponds to a 
   *            connected component and stores the indices of vertices belonging
   *            to it.
   */
  inline
  std::vector<std::vector<int> > getConnectedComponents(const Graph &g)
  {
    std::vector<bool> visited (g.size(), false);
    std::vector<std::vector<int> > CCs;
    
    for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
    {
      // If node has already been visited - skip
      if (visited[vertexId])
        continue;
      
      // Run breadth-first search from current vertex
      std::queue<int> vertexQueue;
      std::vector<int> CC;
      vertexQueue.push(vertexId);
      visited[vertexId] = true;
      
      while (!vertexQueue.empty())
      {
        int curVertex = vertexQueue.front();
        vertexQueue.pop();
        
        CC.push_back(curVertex);
        for (size_t nbrVertexId = 0; nbrVertexId < g[curVertex].size(); nbrVertexId++)
        {
          int nbrVertex = g[curVertex][nbrVertexId];
          
          if (!visited[nbrVertex])
          {
            vertexQueue.push(nbrVertex);
            visited[nbrVertex] = true;
          }
        }
      }
      CCs.push_back(CC);
    }
    
    return CCs;
  }
  
  /** Find the longest path in an undirected acyclic unweighted graph starting
   * from a given vertex. Search is performed using recursive depth first 
   * search.
   * \param[in]   graph             input graph
   * \param[in]   vertex_start      frst vertex in the path
   * \param[in]   visited_vertices  vertices that have already been visited
   * \return a sequence of vertices in the longest path
   */
  inline
  std::vector<int> getLongestPath ( const Graph         &graph,
                                    const int           &vertex_start,
                                    std::vector<bool>   &visited_vertices
                      )
  {
    // Mark start vertex as visited
    visited_vertices[vertex_start] = true;
    
    // Find all adjacent vertices of the start vertex
    std::vector<int> neighbors = graph[vertex_start];
    
    // Remove all neighbours that have already been visited
    std::vector<int>::iterator endIt = std::remove_if(neighbors.begin(), neighbors.end(), [&visited_vertices](int i) { return visited_vertices[i]; });
    neighbors.erase(endIt, neighbors.end());    
    
    std::vector<int> longestPath (0);
    
    // If there are no neighbors that have not been visited this must be a leaf
    // so we return it's index
    if (neighbors.size() == 0)
    {
      longestPath.push_back(vertex_start);
    }
    
    // Otherise call the method on all the neighbors
    else
    {
      // Find all paths from current vertex
      std::vector<std::vector<int> > paths;
      for (std::vector<int>::iterator nbrIdItr = neighbors.begin(); nbrIdItr != neighbors.end(); nbrIdItr++)
      {
        std::vector<int> curPath = getLongestPath(graph, *nbrIdItr, visited_vertices);
        curPath.push_back(vertex_start);
        
        paths.push_back(curPath);
      }
      
      // Return the longest path
      longestPath = *std::max_element(paths.begin(), paths.end(), [](std::vector<int> p1, std::vector<int> p2) {return p1.size() < p2.size();});
    }
      
    return longestPath;
  }
  
  /** Find the longest path in an undirected acyclic unweighted graph.
   * \param[in]   graph             input graph
   * \return a sequence of vertices in the longest path
   */
  inline
  std::vector<int> getLongestPath ( const Graph &graph)
  {
    std::vector<int> leaves = utl::getGraphLeaves(graph);
    std::vector<std::vector<int> > paths(leaves.size());
    for (size_t leafId = 0; leafId < leaves.size(); leafId++)
    {
      std::vector<bool> visitedVertices (graph.size(), false);
      paths[leafId] = utl::getLongestPath(graph, leaves[leafId], visitedVertices);
    }
    
    return *std::max_element(paths.begin(), paths.end(), [](std::vector<int> p1, std::vector<int> p2) {return p1.size() < p2.size();});
  }
  
  /** Datastructure holding the weights of the edges in an adjacency list
   */
  typedef std::vector<std::vector<float> > GraphWeights;
  
  /** Create an empty graph weight datastructure with the same dimensionality as
   * a prototype graph.
   * \param[in] g     graph
   * \return graph weights datastructure
   */
  inline
  GraphWeights createGraphWeights (const Graph &g)
  {
    GraphWeights gw (g.size());
    
    for (size_t vertexId = 0; vertexId < g.size(); vertexId++)
      gw[vertexId].resize(g[vertexId].size());
    
    return gw;
  }
  
  /** Get an edge weight from weighted grapg datastructure
   * \param[in,out]   gw        weighted graph
   * \param[in]       g         corresponding graph
   * \param[in]       v1        first vertex of edge
   * \param[in]       v2        second vertex of edge
   * \param[in]       weight    edge weight
   * \return false if provided edge does not exist in the graph
   */
  inline
  bool getEdgeWeight (const Graph &g, const GraphWeights &gw, const int &v1, const int &v2, float &weight)
  {
    if (v1 > g.size() || v2 > g.size())
    {
      std::cout << "[utl::getEdgeWeight] input edge vertices are out of bounds\n";
      return false;
    }
    
    // Find iterators to elements
    std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
    std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);
    
    // If they are out of bounds - return false
    if ((v1It == g[v2].end()) || (v2It == g[v1].end()))
    {
      std::cout << "[utl::getEdgeWeight] input edge does not exist in the graph or graph is corrupted\n";
      return false;
    }
    
    // Otherwise update the weights
    int v2Id = v2It - g[v1].begin();
    int v1Id = v1It - g[v2].begin();
    
    if (gw[v1][v2Id] != gw[v2][v1Id])
    {
      std::cout << "[utl::getEdgeWeight] edge weights differ for (v1, v2) and (v2, v1)\n";
      return false;
    }
    
    weight = gw[v1][v2Id];
        
    return true;
  }    
  
  /** Add an edge weight to the graph weight datastructure
   * \param[in,out]   gw        weighted graph
   * \param[in]       g         corresponding graph
   * \param[in]       v1        first vertex of edge
   * \param[in]       v2        second vertex of edge
   * \param[in]       weight    edge weight
   * \return false if provided edge does not exist in the graph
   * NOTE: if edge weight already exists it is overwritten
   */
  inline
  bool setEdgeWeight (const Graph &g, GraphWeights &gw, const int &v1, const int &v2, const float &weight)
  {
    if (v1 > g.size() || v2 > g.size())
    {
      std::cout << "[utl::setEdgeWeight] input edge vertices are out of bounds\n";
      return false;
    }
    
    // Find iterators to elements
    std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
    std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);
    
    // If they are out of bounds - return false
    if ((v1It == g[v2].end()) || (v2It == g[v1].end()))
    {
      std::cout << "[utl::setEdgeWeight] input edge does not exist in the graph or graph is corrupted\n";
      return false;
    }
    
    // Otherwise update the weights
    int v2Id = v2It - g[v1].begin();
    int v1Id = v1It - g[v2].begin();
    gw[v1][v2Id] = weight;
    gw[v2][v1Id] = weight;
        
    return true;
  }
    
  /** Add a weighted edge to the weighted graph datastructure. If edge already
   * exists it's weight is overwritten.
   * \param[in,out]   g         graph
   * \param[in,out]   gw        graph weights
   * \param[in]       v1        first vertex of edge
   * \param[in]       v2        second vertex of edge
   * \param[in]       weight    edge weight
   * \return false if provided edge does not exist in the graph
   */
  inline  
  bool addWeightedEdge (Graph &g, GraphWeights &gw, const int &v1, const int &v2, const float &weight)
  {
    // Check that vertices are in bounds
    if (v1 > g.size() || v2 > g.size() || v1 < 0 || v2 < 0)
    {
      std::cout << "[utl::addWeightedEdge] vertex indices are out of bounds." << std::endl;
      return false;
    }
    
    // Check that it is a valid edge
    if (v1 == v2)
    {
      std::cout << "[utl::addWeightedEdge] edge must be between two different vertices." << std::endl;
      return false;
    }
    
    // Find iterators to elements
    std::vector<int>::const_iterator v1It = std::find(g[v2].begin(), g[v2].end(), v1);
    std::vector<int>::const_iterator v2It = std::find(g[v1].begin(), g[v1].end(), v2);

    // If edge doesn't exist create new one
    if (v1It == g[v2].end() && v2It == g[v1].end())
    {
      g[v1].push_back(v2);
      g[v2].push_back(v1);
      gw[v1].push_back(weight);
      gw[v2].push_back(weight);
    }    
    
    // If edge already exists - just update the weight
    else
    {
      int v2Id = v2It - g[v1].begin();
      int v1Id = v1It - g[v2].begin();
      gw[v1][v2Id] = weight;
      gw[v2][v1Id] = weight;      
    }
    
    return true;
  }  
  
  /** Convert graph to a vector of pairs representing edges
   * \param[in]       g         corresponding graph
   * \return vector of pairs where each pair represents an edge
   */
  inline
  void graphWeighted2EdgePairs (const Graph &g, const GraphWeights &w, std::vector<std::pair<int,int> > &edge_pairs, std::vector<float> &edge_pair_weights)
  {
    // Get edge pairs
    edge_pairs = graph2EdgePairs(g);
    
    // Get edge pair weights
    edge_pair_weights.resize(edge_pairs.size());
    for (size_t edgeId = 0; edgeId < edge_pairs.size(); edgeId++)    
    {
      int sourceId = edge_pairs[edgeId].first;
      int targetId = edge_pairs[edgeId].second;
      getEdgeWeight(g, w, sourceId, targetId, edge_pair_weights[edgeId]);
    }
  }
  
  /** Convert graph to a vector of pairs representing edges
   * \param[in]       g         corresponding graph
   * \return vector of pairs where each pair represents an edge
   */
  inline
  bool chechWeightedGraphCorruption(const Graph &g, const GraphWeights &gw)
  {
    // Check vertex sizes
    if (g.size() != gw.size())
    {
      std::cout << "[utl::chechWeightedGraphCorruption] graph and graph weights have different sizes (" << g.size() << ", " << gw.size() << ")" << std::endl;
      return false;
    }
    
    // Check sizes of edges
    bool good = true;
    for (size_t v1 = 0; v1 < g.size(); v1++)
    {
      if (g[v1].size() != gw[v1].size())
      {
        std::cout << "[utl::chechWeightedGraphCorruption] graph edges and graph weights edges have different sizes (" << v1 << ": " << g[v1].size() << ", " << gw[v1].size() << ")" << std::endl;
      }
    }
    
    return good;    
  }
  
}

#endif  // ADJACENCY_LIST_HPP