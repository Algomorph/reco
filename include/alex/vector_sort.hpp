#ifndef VECTOR_SORT_HPP
#define VECTOR_SORT_HPP

#include <vector>
#include <algorithm>

// Source: http://www.alecjacobson.com/weblog/?p=1527

namespace utl
{
  // Act like matlab's [Y,I] = SORT(X)
  // Input:
  //   unsorted  unsorted vector
  // Output:
  //   sorted     sorted vector, allowed to be same as unsorted
  //   index_map  an index map such that sorted[i] = unsorted[index_map[i]]
  template <class T>
  void sort(
      std::vector<T> &unsorted,
      std::vector<T> &sorted,
      std::vector<size_t> &index_map,
      std::string mode = "ascending");

  // Act like matlab's Y = X[I]
  // where I contains a vector of indices so that after,
  // Y[j] = X[I[j]] for index j
  // this implies that Y.size() == I.size()
  // X and Y are allowed to be the same reference
  template< class T >
  std::vector<T> reorder(
    std::vector<T> & unordered, 
    std::vector<size_t> const & index_map);

  ////////////////////////////////////////////////////////////////////////////////
  // Implementation
  ////////////////////////////////////////////////////////////////////////////////

  // Comparison struct used by sort
  // http://bytes.com/topic/c/answers/132045-sort-get-index
  template<class T> struct index_cmp_asc 
  {
    index_cmp_asc(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { 
      return arr[a] < arr[b];
    }
    const T arr;
  };

  template<class T> struct index_cmp_dsc 
  {
    index_cmp_dsc(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { 
      return arr[a] > arr[b];
    }
    const T arr;
  };

  template <class T>
  void sort(
    std::vector<T> & unsorted,
    std::vector<T> & sorted,
    std::vector<size_t> & index_map,
    std::string mode)
  {
    // Original unsorted index map
    index_map.resize(unsorted.size());
    for(size_t i=0;i<unsorted.size();i++)
    {
      index_map[i] = i;
    }
    // Sort the index map, using unsorted for comparison
    if (mode.compare("ascending") == 0)
      sort(
        index_map.begin(), 
        index_map.end(), 
        index_cmp_asc<std::vector<T>& >(unsorted));
      
    else if (mode.compare("descending") == 0)
      sort(
        index_map.begin(), 
        index_map.end(), 
        index_cmp_dsc<std::vector<T>& >(unsorted));    
      
    else
    {
      std::cout << "Unknown sort mode. Must be either \"ascending\" or \"descending.\" Assuming \"ascending\"\n";
      sort(
        index_map.begin(), 
        index_map.end(), 
        index_cmp_asc<std::vector<T>& >(unsorted));    
    }
      
    sorted = reorder(unsorted, index_map);
  }

  // This implementation is O(n), but also uses O(n) extra memory
  template< class T >
  std::vector<T> reorder(
    std::vector<T> & unordered, 
    std::vector<size_t> const & index_map)
  {
    // copy for the reorder according to index_map, because unsorted may also be
    // sorted
    std::vector<T> ordered (index_map.size());
    for(int i = 0; i<index_map.size();i++)
    {
      ordered[i] = unordered[index_map[i]];
    }
    
    return ordered;
  }
}

#endif // VECTOR_SORT_HPP