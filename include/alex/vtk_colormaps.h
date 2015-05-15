#ifndef VTK_COLORMAPS_H
#define VTK_COLORMAPS_H

// Includes
#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>
#include <vtk_colormaps_lut.hpp>
#include <eigen3/Eigen/Dense>

namespace utl
{
  typedef std::vector<double> Colour;
  typedef std::vector<Colour> Colours;
  
  class Colormap
  {    
    public:
      
      enum COLORMAP_TYPE {VTK_DEFAULT, JET, GRAYSCALE};
      
      // Constructors
      Colormap ()
        : colormapType_ (JET)
        , minVal_ (std::numeric_limits<double>::quiet_NaN())
        , maxVal_ (std::numeric_limits<double>::quiet_NaN())
      {};
      
      Colormap (COLORMAP_TYPE colormapType)
        : colormapType_ (colormapType)
        , minVal_ (std::numeric_limits<double>::quiet_NaN())
        , maxVal_ (std::numeric_limits<double>::quiet_NaN())
      {};
      
      Colormap (COLORMAP_TYPE colormapType, double minVal, double maxVal)
        : colormapType_ (colormapType)
        , minVal_ (minVal)
        , maxVal_ (maxVal)
      {};    
            
      // Set colormap type
      void setColormapType (COLORMAP_TYPE colormapType);
      
      // Manually set range limits
      void setRangeLimits (double minVal, double maxVal);
      
      // Reset range limits to undefined
      void resetRangeLimits();

      // Set range limits from data
      template <typename Scalar>
      void setRangeLimitsFromData (const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data);

      // Set range limits from data
      template <typename Scalar>
      void setRangeLimitsFromData (const std::vector<Scalar> &data);

      // Get vtk lookup table representing the colormap      
      vtkSmartPointer<vtkLookupTable> getColorLookupTable() const;
      
      // Map each element of a scalar vector to RGB values in [0,1] range given the colourmap
      template <typename Scalar>
      Colours getColoursFromData (const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data);

      // Map each element of a scalar vector to RGB values in [0,1] range given the colourmap
      template <typename Scalar>
      Colours getColoursFromData (const std::vector<Scalar> &data);
      
    private:
      
      static const int COLORMAP_NUM_DIVISIONS = 256;
      COLORMAP_TYPE colormapType_;
      double minVal_;
      double maxVal_;
      
      // Get specific colormaps
      vtkSmartPointer<vtkLookupTable> getLUT_Jet () const;
      vtkSmartPointer<vtkLookupTable> getLUT_VtkDefault () const;
      vtkSmartPointer<vtkLookupTable> getLUT_Grayscale () const;
      
  };
}
//   void resetColormapInfoRangeLimits (ColormapInfo &colormap_info)
//   {
//     colormap_info
//   }
  
//   void getColormapVtkDefault  (vtkSmartPointer<vtkLookupTable> &colormap);
//   void getColormapJet         (vtkSmartPointer<vtkLookupTable> &colormap);
//   void getColormapGrayscale   (vtkSmartPointer<vtkLookupTable> &colormap);
// 
//   /** \brief Generate a vtk lookuptable for a given colormap
//     * \param[out] colormap  pointer to a vtk lookup table with the colormap
//     * \param[in]  colormap_type colormap type
//     * \return false if colormap type does not exist
//     */
//   bool getColormap (vtkSmartPointer<vtkLookupTable> &colormap, const COLORMAP_TYPE &colormap_type = JET)
//   {  
//     switch (colormap_type)
//     {
//       case VTK_DEFAULT:      
//         getColormapVtkDefault(colormap);
//         break;
// 
//       case GRAYSCALE:
//         getColormapGrayscale(colormap);
//         break;
//         
//       case JET:
//         getColormapJet(colormap);
//         break;
//         
//       default:
//         std::cout << "[utl::getColormap] Unknown colormap\n";
//         return false;
//         
//     }
//     
//     return true;
//   }
//   
// 
//   /** \brief Generate a default vtk lookup table
//     * \param[out] colormap  pointer to a vtk lookup table with the colormap
//     */  
//   void getColormapVtkDefault(vtkSmartPointer<vtkLookupTable> &colormap)
//   {
//     colormap = vtkSmartPointer<vtkLookupTable>::New();
//     colormap->Build();
//   }
//   
//   /** \brief Generate a lookup table for grayscale colormap
//     * \param[out] colormap  pointer to a vtk lookup table with the colormap
//     */  
//   void getColormapGrayscale(vtkSmartPointer<vtkLookupTable> &colormap)
//   {
//     colormap = vtkSmartPointer<vtkLookupTable>::New();
//     colormap->SetNumberOfTableValues(COLORMAP_NUM_DIVISIONS);
//   
//     for (size_t i = 0; i < COLORMAP_NUM_DIVISIONS; i++)
//     {
//       double value = 1.0/(COLORMAP_NUM_DIVISIONS-1) * i;
//       colormap->SetTableValue(i,  value, value, value);
//     }    
//   }
// 
//   
//   /** \brief Generate lookup table for jet colormap
//     * \param[out] colormap  pointer to a vtk lookup table with the colormap
//     */  
//   void getColormapJet(vtkSmartPointer<vtkLookupTable> &colormap)
//   {
//     colormap = vtkSmartPointer<vtkLookupTable>::New();
//     colormap->SetNumberOfTableValues(COLORMAP_NUM_DIVISIONS);
// 
//     for (size_t i = 0; i < COLORMAP_NUM_DIVISIONS; i++)
//     {
//       colormap->SetTableValue(i,  JET_LUT[i][0], JET_LUT[i][1], JET_LUT[i][2]);
//     }    
//   }
//   
//   /** \brief Given an Eigen column vector of scalars generate an RGB colour in [0,1] range for each point according to some colormap
//     * \param[in]  data            vector containing the data
//     * \param[in,out]  colormap_info   colormap information datastructure. If minimum/maximum data range values are calculated automatically if they are set to NaN
//     * \param[out] colours         output vector of colours
//     */
//   template <typename Scalar>
//   inline
//   void coloursFromData( const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data,
//                         ColormapInfo &colormap_info,
//                         std::vector<std::vector<double> > &colours
//                       )
//   {
//     // Generate the max and min range of the data
//     double maxVal, minVal;
//     if (isnan(colormap_info.maxVal_))
//       maxVal = static_cast<double>(data.maxCoeff());
//     else
//       maxVal = static_cast<double>(colormap_info.maxVal_);
//       
//     if (isnan(colormap_info.minVal_))
//       minVal = static_cast<double>(data.minCoeff());
//     else
//       minVal = static_cast<double>(colormap_info.minVal_);
//     
//     // Create the color map
//     vtkSmartPointer<vtkLookupTable> colorLookupTable;
//     getColormap(colorLookupTable, colormap_info.colormapType_);
//     colorLookupTable->SetTableRange(minVal, maxVal);
//     
//     // Generate colours
//     colours.resize(data.size());
//     for (size_t i = 0; i < data.size(); i++)
//     {
//       double rgb[3];
//       colorLookupTable->GetColor(data[i], rgb);
//       std::vector<double> rgbVec (3);
//       rgbVec[0] = rgb[0];
//       rgbVec[1] = rgb[1];
//       rgbVec[2] = rgb[2];
//       colours[i] = rgbVec;
//     }
//   }  
// 
//   /** \brief Given an Eigen column vector of scalars generate an RGB colour in [0,1] range for each point according to some colormap
//     * \param[in]  data            vector containing the data
//     * \param[in,out]  colormap_info   colormap information datastructure. If minimum/maximum data range values are calculated automatically if they are set to NaN
//     * \param[out] colours         output vector of colours
//     */
//   template <typename Scalar>
//   inline
//   void coloursFromData( const std::vector<Scalar> &data,
//                         ColormapInfo &colormap_info,
//                         std::vector<std::vector<double> > &colours
//                       )
//   {
//     // Generate the max and min range of the data
//     double maxVal, minVal;
//     if (isnan(colormap_info.maxVal_))
//       maxVal = static_cast<double>(*std::max_element(data.begin(), data.end()));
//     else
//       maxVal = static_cast<double>(colormap_info.maxVal_);
//       
//     if (isnan(colormap_info.minVal_))
//       minVal = static_cast<double>(*std::min_element(data.begin(), data.end()));
//     else
//       minVal = static_cast<double>(colormap_info.minVal_);
//     
//     // Create the color map
//     vtkSmartPointer<vtkLookupTable> colorLookupTable;
//     getColormap(colorLookupTable, colormap_info.colormapType_);
//     colorLookupTable->SetTableRange(minVal, maxVal);
//     
//     // Generate colours
//     colours.resize(data.size());
//     for (size_t i = 0; i < data.size(); i++)
//     {
//       double rgb[3];
//       colorLookupTable->GetColor(data[i], rgb);
//       std::vector<double> rgbVec (3);
//       rgbVec[0] = rgb[0];
//       rgbVec[1] = rgb[1];
//       rgbVec[2] = rgb[2];
//       colours[i] = rgbVec;
//     }
//   }
// }

#endif  // VTK_COLORMAPS_H