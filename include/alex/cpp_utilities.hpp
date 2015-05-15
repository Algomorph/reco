#ifndef CPP_UTILITIES_HPP
#define CPP_UTILITIES_HPP

// STD includes
#include <iostream>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/regex.hpp>

// Other
#include <alphanum.hpp>

namespace utl
{
  
  /** \brief Replace all occurences of a substring with a new substring
    * \param[in,out] string_in string which is modified
    * \param[in] substring_orig substring that needs to be replaced
    * \param[in] substring_new new substring
    */
  inline
  void replaceSubstring(std::string &string_in, const std::string &substring_orig, const std::string &substring_new)
  {
    std::string::size_type n = 0;
    while ( ( n = string_in.find( substring_orig, n ) ) != std::string::npos )
    {
        string_in.replace( n, substring_orig.size(), substring_new );
        n += substring_new.size();
    }  
  }

  /** \brief An analogue of the MATLAB dir function
    * \param[in] dir_path input directory
    * \param[out] content_list a vector of strings representing names of files/folders in the directory, sorted alphanumerically
    */
  inline
  void dir(const std::string &dir_path, std::vector<std::string> &content_list)
  {
    std::string dirPath (dir_path);
    
    // First normalize the beginning of the path
    // If directory path does not start with '/', './' or '../' append './'
    if  (!(((dirPath.length() >= 1) && (dirPath.substr(0,1) == "/")) || 
        ((dirPath.length() >= 2) && (dirPath.substr(0,2) == "./")) ||
        ((dirPath.length() >= 3) && (dirPath.substr(0,3) == "../"))))
    {
      dirPath.insert(0, "./");
    }
      
    // Find the rightmost '/' delimeter and split along it
    std::string left, right;
    size_t found = dirPath.find_last_of("/");
    if (found == std::string::npos)
    {
      std::cout << "[dir] something wrong\n";
      exit(EXIT_FAILURE);
    }
    else
    {
      left = dirPath.substr(0, found+1);
      right = dirPath.substr(found+1);
    }
    
    // Now analyze the right part
    std::string fileFormatStr;
    
    // If right part is empty list everything in left
    if (right.empty())
    {
      fileFormatStr = "*";
      dirPath = left;
    }
    
    // If right side containts a wildcard or an extension list everything in left that matches
    else if ((right.find("*") != std::string::npos) || (boost::filesystem::path(right).has_extension()))
    {
      fileFormatStr = right;
      dirPath = left;
    }
    
    // If right is a directory append it to left and list everything
    else if (!boost::filesystem::path(right).has_extension())
    {
      fileFormatStr = "*";
      dirPath = left + right;
    }

    // Generate regex expression
    std::string regexExpression (fileFormatStr);
    replaceSubstring(regexExpression, ".", "[.]");      // Replace all occurences of '.' with '[.]'
    replaceSubstring(regexExpression, "*", ".*");       // Replace all occurences of '*' with '.*'
    boost::regex filenameFormatRgx(regexExpression);  
    
    // List
    content_list.resize(0);
    for (boost::filesystem::directory_iterator file_it(dirPath), end; file_it != end; ++file_it)
    {
      std::string curFileName = file_it->path().leaf().string();                          // Get current file name
      
      if (boost::regex_match(curFileName, filenameFormatRgx))                               // Check if it matches the filename format   
        content_list.push_back(curFileName);
    }
    
    // Sort the list
    std::sort(content_list.begin(), content_list.end(), doj::alphanum_less<std::string>());    
  }

  /** \brief Split a string by a delimeter
    * \param[in] path string to be separated
    * \param[in] delimiter delimiter
    * \return a vector of delimited substrings
    * \NOTE: http://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
    */
  inline
  std::vector<std::string> splitString (const std::string &line, const std::string delimiter = " ")
  {
    std::vector<std::string> substrings;
    
    auto start = 0U;
    auto end = line.find(delimiter);
    while (end != std::string::npos)
    {
      substrings.push_back(line.substr(start, end - start));
      start = end + delimiter.length();
      end = line.find(delimiter, start);
    }
    
    end = line.length();
    substrings.push_back(line.substr(start, end - start));
    
    return substrings;
  }

  /** \brief Add trailing slash to pathname if it is missing
    * \param[in] pathname file path
    * \return path with trailing slash
    */
  inline
  std::string addTrailingSlash (const std::string &pathname)
  {
    if (boost::filesystem::path(pathname).has_extension())
      return pathname;
    if (!pathname.empty() && pathname.back() != '/')
      return pathname + '/';
    else
      return pathname;
  }
  
  /** \brief Add trailing slash to pathname if it is missing
    * \param[in] pathname file path
    * \return path with trailing slash
    */
  inline
  std::string removeTrailingSlash (const std::string &pathname)
  {
    if (!pathname.empty() && pathname.back() == '/')
      return pathname.substr(0, pathname.size()-1);
    else
      return pathname;
  }
  
  /** \brief Find the parent directory of a path
    * \param[in] path path
    * \return parent directory
    */
  inline
  std::string getParentDir (const std::string &path)
  {
    std::string parentDir = boost::filesystem::path(removeTrailingSlash(path)).parent_path().string();
    if (parentDir == "")
      parentDir = "./";
    
    return parentDir;
  }
  
  /** \brief If input is a filename - return filename without preceeding path. 
   *  If input is path - return last directory in the path
    * \param[in] path path
    * \return filename
    */
  inline
  std::string getBasename (const std::string &path)
  {
    if (boost::filesystem::path(path).has_extension())
      return boost::filesystem::path(path).filename().string();
    else if (!path.empty() && path.back() != '/')
      return boost::filesystem::path(path).stem().string();
    else
      return boost::filesystem::path(path.substr(0, path.length()-1)).stem().string();
  }

  /** \brief Get filename without extension from the path
    * \param[in] path path
    * \return filename without extension
    */
  inline
  std::string getBasenameNoExtension (const std::string &path)
  {
    return boost::filesystem::path(path).stem().string();
  }
  
  /** \brief Check if a path exists or not
    * \return true if path exists
    */
  inline
  bool exists (const std::string &path)
  {
    return boost::filesystem::exists(path);
  }

  /** \brief Check if a path is an existing file
    * \return true if path is a filename
    */
  inline
  bool isFile (const std::string &path)
  {
    return (boost::filesystem::is_regular_file(boost::filesystem::path(path)));    
  }
  
  /** \brief Check if a path is a directory that exists
    * \return true if path is a directory
    */
  inline
  bool isDirectory (const std::string &path)
  {
    return (boost::filesystem::is_directory(boost::filesystem::path(path)));
  }  
  
  /** \brief Delete a directory and all of its contents
    * \param[in] path path
    * \return false if directory does not exist or could not delete directory
    */
  inline
  bool deleteDir (const std::string &path)
  {
    if (!boost::filesystem::remove_all(path))
    {
      std::cout << "[utl::deleteDir] could not delete directory '" << path << "'\n";
      return false;
    }
    
    return true;
  }
  
  /** \brief Create a directory and all of its contents
    * \param[in] path path
    * \return false if directory already exists or could not create directory
    */
  inline
  bool createDir (const std::string &path)
  {    
    if (!boost::filesystem::create_directory(boost::filesystem::path(path)))
    {
      std::cout << "[utl::createDir] Could not create directory '" << path << "'\n";
      return false;
    }
    
    return true;
  }
  
  /** \brief Join to paths to generate new path. Inspired by MATLAB's fullfile
    * \param[in] path1 first path
    * \param[in] path2 second path
    * \return joined path
    */
  inline
  std::string fullfile (const std::string &path1, const std::string &path2)
  {
    boost::filesystem::path joinedPath = boost::filesystem::path(path1) / boost::filesystem::path(path2);
    return joinedPath.string();
  }  
  
  /** \brief Clamp a value from above and below
    * \param[in] value value to be clamped
    * \param[in] minValue minimum value
    * \param[in] maxValue maximum value
    * \return bounded value
    */        
  template <typename Scalar>
  inline
  Scalar clampValue(const Scalar &value, const Scalar &minValue, const Scalar &maxValue)
  {
    Scalar valueBounded = value;
    valueBounded = std::max(valueBounded, minValue);
    valueBounded = std::min(valueBounded, maxValue);
    return valueBounded;
  }
  
  /** \brief Compute a occurences of values in a vector
    * \param[in] values a vector of values
    * \return a vector of pairs where first value is value and second is 
    * number of occurences of the value
    * \note this implementation is inefficient 
    */        
  template <typename Scalar>
  inline
  std::vector<std::pair<Scalar, int> > vectorCount(const std::vector<Scalar> &values)
  { 
    // First get all the unique values of the vector
    std::vector<Scalar> uniqueValues = values;
    std::sort(uniqueValues.begin(), uniqueValues.end());
    uniqueValues.erase(std::unique(uniqueValues.begin(), uniqueValues.end()), uniqueValues.end());
    
    // Count the occurence of each value
    std::vector<std::pair<Scalar, int> > counts (uniqueValues.size());
    
    for (size_t i = 0; i < uniqueValues.size(); i++)
    {
      int count = std::count(values.begin(), values.end(), uniqueValues[i]);
      counts[i] = std::pair<int, int>(uniqueValues[i], count);
    }
    return counts;
  }
  
  /** \brief Find an intersection between two vectors. The return vector contains unique values that are present in both vectors.
    * \param[in] v1 first vector
    * \param[in] v2 second vector
    * \return vector intersection
    */        
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorIntersection(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    
    std::vector<int> v_intersection;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());
    std::set_intersection(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_intersection));
    
    return v_intersection;
  }

  /** \brief Find a union of two vectors. The return vector contains unique values that are present in both vectors.
    * \param[in] v1 first vector
    * \param[in] v2 second vector
    * \return vector union
    */        
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorUnion(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    
    std::vector<int> v_union;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());  
    v1_sorted.erase(std::unique(v1_sorted.begin(), v1_sorted.end()), v1_sorted.end());
    v2_sorted.erase(std::unique(v2_sorted.begin(), v2_sorted.end()), v2_sorted.end());
    std::set_union(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_union));
    
    return v_union;
  }

  /** \brief Find a difference between two vectors. The return vector contains unique values that are present in first vector but not in the second
    * \param[in] v1 first vector
    * \param[in] v2 second vector
    * \return vector difference
    */        
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorDifference(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    std::vector<int> v_difference;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());  
    v1_sorted.erase(std::unique(v1_sorted.begin(), v1_sorted.end()), v1_sorted.end());
    v2_sorted.erase(std::unique(v2_sorted.begin(), v2_sorted.end()), v2_sorted.end());  
    std::set_difference(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_difference));
      
    return v_difference;
  }
  
  /** \brief Remove all instances of an element from vector
    * \param[in] v  vector
    * \param[in] el element to be removed
    */        
  template <typename TObject>
  inline
  void removeElement (std::vector<TObject> &v, const TObject &el)
  {
    v.erase(std::remove(v.begin(), v.end(), el), v.end());
  }
  
  /** \brief Make vector unique value. Values are sorted.
    * \param[in,out] v  vector
    */        
  template <typename TObject>
  inline
  void uniqueVector (std::vector<TObject> &v)
  {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  
  /** \brief Get maximum value of a vector of scalars
    * \param[in] v  vector
    * \return maximum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMax (const std::vector<Scalar> &v)
  {    
    return *std::max_element(v.begin(), v.end());
  }  
  
  /** \brief Get minimum value of a vector of scalars
    * \param[in] v  vector
    * \return minimum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMin (const std::vector<Scalar> &v)
  {    
    return *std::min_element(v.begin(), v.end());
  }  

  /** \brief Remove elements from vector given an int vector
    * \param[in] v  vector
    * \param[in] indices int vector where each element represents the index of an element in the original vector that needs to be kept
    * \return filtered vector
    */        
  template <typename TObject>
  inline
  std::vector<TObject> vectorFilter (const std::vector<TObject> &v, const std::vector<int> &indices)
  {
    // Check that indices are within the range of the vector size
    if (vectorMax(indices) > v.size()-1 || vectorMin(indices) < 0)
    {
      std::cout << "[utl::vectorFilter] indices are outside vector size range (vector size:" 
                << v.size() << ", indices range: (" << vectorMax(indices) << ", " << vectorMin(indices) << "))\n";
      return std::vector<TObject>(0);
    }
    
    // Filter
    std::vector<TObject> v_filtered;
    for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); it++)
      v_filtered.push_back(v[*it]);
   
    return v_filtered;
  }
  
  /** \brief Remove elements from vector given a bool vector
    * \param[in] v  vector
    * \param[in] mask bool vector where 'false' indicates that corresponding element must be removed
    * \return filtered vector
    */        
  template <typename TObject>
  inline
  std::vector<TObject> vectorFilter (const std::vector<TObject> &v, const std::vector<bool> &mask)
  {
    // Check that mask 
    if (v.size() != mask.size())
    {
      std::cout << "[utl::vectorFilter] vector and mask must be the same size (" << v.size() << ", " << mask.size() << ")\n";
      return std::vector<TObject>(0);
    }
    
    // Filter
    std::vector<TObject> v_filtered;
    for (size_t elId = 0; elId < mask.size(); elId++)
      if (mask[elId])
        v_filtered.push_back(v[elId]);
   
    return v_filtered;
  }
}



#endif      // CPP_UTILITIES_HPP
