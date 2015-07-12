
#define PY_ARRAY_UNIQUE_SYMBOL reco_ARRAY_API
#include <boost/python.hpp>
#include <reco/python/cv_boost_converter.hpp>
namespace reco {
namespace python {

using namespace boost::python;

/**
 * Example function. Basic inner matrix product using explicit matrix conversion.
 * @param left left-hand matrix operand (NdArray required)
 * @param right right-hand matrix operand (NdArray required)
 * @return an NdArray representing the dot-product of the left and right operands
 */
PyObject* dot(PyObject *left, PyObject *right) {

	cv::Mat leftMat, rightMat;
	leftMat = reco::python::fromNDArrayToMat(left);
	rightMat = reco::python::fromNDArrayToMat(right);
	auto c1 = leftMat.cols, r2 = rightMat.rows;
	// Check that the 2-D matrices can be legally multiplied.
	if (c1 != r2){
		PyErr_SetString(PyExc_TypeError,
				"Incompatible sizes for matrix multiplication.");
		throw_error_already_set();
	}
	cv::Mat result = leftMat * rightMat;
	PyObject* ret = reco::python::fromMatToNDArray(result);
	return ret;
}

//This example uses Mat directly, but we won't need to worry about the conversion
/**
 * Example function. Basic inner matrix product using implicit matrix conversion.
 * @param leftMat left-hand matrix operand
 * @param rightMat right-hand matrix operand
 * @return an NdArray representing the dot-product of the left and right operands
 */
cv::Mat dot2(cv::Mat leftMat, cv::Mat rightMat) {
	auto c1 = leftMat.cols, r2 = rightMat.rows;
	if (c1 != r2) {
		PyErr_SetString(PyExc_TypeError,
				"Incompatible sizes for matrix multiplication.");
		throw_error_already_set();
	}
	cv::Mat result = leftMat * rightMat;

	return result;
}

static void init_ar(){
	Py_Initialize();

	import_array();
}

BOOST_PYTHON_MODULE(reco){
	//using namespace XM;
	init_ar();

	//initialize converters
	to_python_converter<cv::Mat,
			reco::python::matToNDArrayBoostConverter>();
	reco::python::matFromNDArrayBoostConverter();

	//expose module-level functions
	def("dot", dot);
	def("dot2", dot2);
}

} //end namespace python
} //end namespace reco
