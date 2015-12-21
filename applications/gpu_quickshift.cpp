#include <cuda_runtime.h>

#include <reco/cuda/helper_cuda.h>
#include <reco/segmentation/quickshift_common.h>
#include <reco/segmentation/exception.h>
#include <reco/segmentation/image.h>
#include <reco/utils/cpp_exception_util.h>
#include <reco/utils/debug_util.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <fstream>

void write_image(image_t im, const char * filename) {
	/********** Copy from matlab style **********/
	Image IMGOUT(im.K > 1 ? Image::RGB : Image::L, im.N2, im.N1);
	for (int k = 0; k < im.K; k++)
		for (int col = 0; col < im.N2; col++)
			for (int row = 0; row < im.N1; row++)
					{
				/* Row transpose */
				unsigned char * pt = IMGOUT.getPixelPt(col, im.N1 - 1 - row);
				/* scale 0-255 */
				pt[k] = (unsigned char) (im.I[row + col * im.N1 + k * im.N1 * im.N2] / 32 * 255);
			}

	/********** Write image **********/
	std::ofstream ofs(filename, std::ios::binary);
	if (!ofs) {
		throw Exception("Could not open the file");
	}
	ofs << IMGOUT;
}

void imseg_cv(const cv::Mat& orig, cv::Mat& out, int * flatmap) {
	/********** Mean Color **********/
	int layer_size = orig.rows * orig.cols;
	if (out.empty()) {
		out = cv::Mat(orig.size(), orig.type());
	} else if (out.rows != orig.rows || out.cols != orig.cols || out.type() != orig.type()) {
		err2(std::invalid_argument, "Output matrix type or size not matching the input matrix.");
	}
	float * counts = (float *) calloc(layer_size, sizeof(float));
	if (orig.type() != CV_32F && orig.type() != CV_32FC3) {
		err2(std::invalid_argument, "Expecting a continous matrix of type CV_32F or CV_32FC3");
	}

	//cv::Mat meancolor = cv::Mat(orig.size(), orig.type());
	cv::Mat meancolor = cv::Mat::zeros(orig.size(), orig.type());

	const float* data = orig.ptr<float>();
	float* meancolor_data = reinterpret_cast<float*>(meancolor.data);
	float* out_data = reinterpret_cast<float*>(out.data);
	int n_channels = orig.channels();

	for (int p = 0; p < layer_size; p++) {
		counts[flatmap[p]]++;
		for (int k = 0; k < orig.channels(); k++){
			meancolor_data[flatmap[p]*n_channels + k] += data[p*n_channels+k];
		}
	}

	int roots = 0;
	for (int p = 0; p < layer_size; p++) {
		if (flatmap[p] == p){
			roots++;
		}
	}
	printf("Roots: %d\n", roots);

	int nonzero = 0;
	for (int p = 0; p < layer_size; p++) {
		if (counts[p] > 0) {
			nonzero++;
			for (int k = 0; k < orig.channels(); k++){
				meancolor_data[p*n_channels + k] /= counts[p];
			}
		}
	}
	if (roots != nonzero)
		printf("Nonzero: %d\n", nonzero);
	assert(roots == nonzero);
	/********** Create output image **********/
	for (int p = 0; p < layer_size; p++) {
		counts[flatmap[p]]++;
		for (int k = 0; k < orig.channels(); k++){
			out_data[p*n_channels+k] = meancolor_data[flatmap[p]*n_channels + k];
		}
	}
	free(counts);
}

image_t imseg(image_t im, int * flatmap) {
	/********** Mean Color **********/
	float * meancolor = (float *) calloc(im.N1 * im.N2 * im.K, sizeof(float));
	float * counts = (float *) calloc(im.N1 * im.N2, sizeof(float));

	for (int p = 0; p < im.N1 * im.N2; p++) {
		counts[flatmap[p]]++;
		for (int k = 0; k < im.K; k++)
			meancolor[flatmap[p] + k * im.N1 * im.N2] += im.I[p + k * im.N1 * im.N2];
	}

	int roots = 0;
	for (int p = 0; p < im.N1 * im.N2; p++)
			{
		if (flatmap[p] == p)
			roots++;
	}
	printf("Roots: %d\n", roots);

	int nonzero = 0;
	for (int p = 0; p < im.N1 * im.N2; p++){
		if (counts[p] > 0){
			nonzero++;
			for (int k = 0; k < im.K; k++){
				meancolor[p + k * im.N1 * im.N2] /= counts[p];
			}
		}
	}
	if (roots != nonzero)
		printf("Nonzero: %d\n", nonzero);
	assert(roots == nonzero);

	/********** Create output image **********/
	image_t imout = im;
	imout.I = (float *) calloc(im.N1 * im.N2 * im.K, sizeof(float));
	for (int p = 0; p < im.N1 * im.N2; p++)
		for (int k = 0; k < im.K; k++)
			imout.I[p + k * im.N1 * im.N2] = meancolor[flatmap[p] + k * im.N1 * im.N2];

	free(meancolor);
	free(counts);

	return imout;
}

/**
 * Generate a flat cluster map from a neighbor map
 * @param map - neighbor map, where each pixel indicates one of the neighbors in its cluster
 * @param size - total size of the neighbor map
 * @return
 */
int * map_to_flatmap(float * map, unsigned int size) {
	/********** Flatmap **********/
	int *flatmap = (int *) malloc(size * sizeof(int));
	for (unsigned int p = 0; p < size; p++){
		flatmap[p] = map[p];
	}

	bool changed = true;
	while (changed){
		changed = false;
		for (unsigned int p = 0; p < size; p++){
			changed = changed || (flatmap[p] != flatmap[flatmap[p]]);
			flatmap[p] = flatmap[flatmap[p]];
		}
	}

	/* Consistency check */
	for (unsigned int p = 0; p < size; p++)
		assert(flatmap[p] == flatmap[flatmap[p]]);

	return flatmap;
}

void image_to_matlab(Image & IMG, image_t & im){
	/********** Convert image to MATLAB style representation **********/
	im.N1 = IMG.getHeight();
	im.N2 = IMG.getWidth();
	im.K = IMG.getPixelSize();
	im.I = (float *) calloc(im.N1 * im.N2 * im.K, sizeof(float));
	for (int k = 0; k < im.K; k++){
		for (int col = 0; col < im.N2; col++){
			for (int row = 0; row < im.N1; row++){
				unsigned char * pt = IMG.getPixelPt(col, im.N1 - 1 - row);
				im.I[row + col * im.N1 + k * im.N1 * im.N2] = 32. * pt[k] / 255.; // Scale 0-32
			}
		}
	}
}

int main(int argc, char ** argv) {
	/** Define and parse the program options*/
	namespace fs = boost::filesystem;
	namespace po = boost::program_options;
	using namespace std;

	float sigma, tau;
	int device;

	std::string file;
	std::string mode;
	std::string out_path;
	bool use_cv;
	po::variables_map vm;
	po::options_description regular_options("Options");
	regular_options.add_options()
	("help,h", "Print help messages")
	("file,f", po::value<string>(&file)->default_value("flowers2.pnm"), "Input file")
	("mode,m", po::value<string>(&mode)->default_value("gpu"), "Input mode. One of [gpu,cpu].")
	("sigma,s", po::value<float>(&sigma)->default_value(6.0f), "Sigma parameter for quickshift.")
	("tau,t", po::value<float>(&tau)->default_value(10.0f), "Tau parameter for quickshift.")
	("device,d", po::value<int>(&device)->default_value(-1),
			"Device. Default: device with max. Gflops.")
	("output,o", po::value<string>(&out_path)->default_value(""), "Output path.")
	("cv", po::bool_switch(&use_cv)->default_value(false), "use OpenCV");

	try {
		po::store(po::command_line_parser(argc, argv)
				.options(regular_options).run(), vm); // can throw

		/** --help option
		 */
		if (vm.count("help")) {
			std::cout << "Video background subtractor." << std::endl
					<< regular_options << std::endl;
			return 0;
		}

		po::notify(vm); // throws on error, so do after help in case
						// there are any problems
	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return -2;
	}

	//Use command-line specified CUDA device, otherwise use device with highest Gflops/s
	if (device > -1) {
		cudaSetDevice(device);
	} else {
		cudaSetDevice(gpuGetMaxGflopsDeviceId());
	}

	std::string modes[1];
	modes[0] = mode;
	int nmodes = 1;

	char outfile[1024];
	/********** Read image **********/
	int width, height;
	image_t im;
	cv::Mat img;
	//const float load_factor = 1.0;
	const float load_factor = 32.0 / 255.0;
	const float save_factor = 1.0 / load_factor;
	if (use_cv) {
		cv::Mat tmp;
		tmp = cv::imread(file, cv::IMREAD_UNCHANGED);

		if (tmp.channels() == 1) {
			tmp.convertTo(img, CV_32F, load_factor);
		} else {
			tmp.convertTo(img, CV_32FC3, load_factor);
		}

		width = img.cols;
		height = img.rows;
	} else {
		Image IMG;

		std::ifstream ifs(file, std::ios::binary);
		if (!ifs) {
			throw Exception("Could not open the file");
		}
		ifs >> IMG;

		image_to_matlab(IMG, im);

		height = im.N1;
		width = im.N2;
	}
	std::chrono::time_point<std::chrono::system_clock> start_total, end_total;
	start_total = std::chrono::system_clock::now();
	/********** CUDA setup **********/
	float *map, *E, *gaps;
	int * flatmap;
	image_t imout;

	map = (float *) calloc(width * height, sizeof(float));
	gaps = (float *) calloc(width * height, sizeof(float));
	E = (float *) calloc(width * height, sizeof(float));

	for (int m = 0; m < nmodes; m++) {
		std::chrono::time_point<std::chrono::system_clock> start_dev, end_dev;
		start_dev = std::chrono::system_clock::now();

		/********** Quick shift **********/
		printf("Mode: %s\n", modes[m].c_str());
		if (modes[m] == "cpu") {
			if (use_cv) {
				err2(std::invalid_argument, "Not currently supported!");
			} else {
				quickshift(im, sigma, tau, map, gaps, E);
			}
		} else if (modes[m] == "gpu") {
			if (use_cv) {
				quickshift_gpu_cv(img, sigma, tau, map, gaps, E);
			} else {
				quickshift_gpu(im, sigma, tau, map, gaps, E);
			}
		} else {
			assert(0 && "Unrecognized mode line");
		}

		end_dev = std::chrono::system_clock::now();

		/* Consistency check */
		for (int p = 0; p < width * height; p++)
			if (map[p] == p)
				assert(gaps[p] == INF);

		flatmap = map_to_flatmap(map, width * height);

		/*Output file name*/
		sprintf(outfile, "%s", file.c_str());
		char * c = strrchr(outfile, '.');
		if (c)
			*c = '\0';
		sprintf(outfile, "%s-%s.pnm", outfile, modes[m].c_str());

		if (use_cv) {
			cv::Mat out, out_tmp;
			imseg_cv(img, out, flatmap);
			if (out.channels() == 1) {
				out.convertTo(out_tmp, CV_8UC1, save_factor);
			} else {						//assume 3 channels
				out.convertTo(out_tmp, CV_8UC3, save_factor);
			}
			if (out_path != "") {
				cv::imwrite(out_path.c_str(), out_tmp);
			} else {
				cv::imwrite(outfile, out_tmp);
			}
		} else {
			imout = imseg(im, flatmap);
			if (out_path != "") {
				write_image(imout, out_path.c_str());
			} else {
				write_image(imout, outfile);
			}
			free(imout.I);
			free(im.I);
		}

		free(flatmap);

		std::chrono::duration<double> mode_seconds = end_dev - start_dev;
		printf("Time: %f s\n\n\n", mode_seconds.count());
	}

	end_total = std::chrono::system_clock::now();
	std::chrono::duration<double> total_seconds = end_total - start_total;
	printf("Total time: %f s\n", total_seconds.count());

	/********** Cleanup **********/
	free(map);
	free(E);
	free(gaps);
}
