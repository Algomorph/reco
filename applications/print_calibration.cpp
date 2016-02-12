/*
 * print_calibration.cpp
 *
 *  Created on: Feb 12, 2016
 *      Author: Gregory Kramida
 *   Copyright: 2016 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <reco/calib/opencv_rectifier.hpp>

namespace
{
const size_t error_in_command_line = 2;
const size_t success = 0;

} // namespace

namespace internal{

namespace fs = boost::filesystem;
namespace cal = reco::calib;

class print_calibration_app {
public:
	print_calibration_app(fs::path path) : rectifier(path.string()) {}
	virtual ~print_calibration_app() {
	}
	int run() {
		rectifier.print();
		return success;
	}
private:
	cal::opencv_rectifier rectifier;
};
}

int main(int argc, char** argv) {
	/** Define and parse the program options*/
	namespace po = boost::program_options;
	namespace fs = boost::filesystem;


	using namespace std;
	po::variables_map vm;

	po::options_description regular_options("Options");
	vector<string> input_files;
	string path;

	regular_options.add_options()
	("help", "Print help messages")
	("path,p", po::value<string>(&path)->required(), "Path to the calibration file.");

	po::positional_options_description positional_options;
	positional_options.add("path", 1);

	try {
		po::store(po::command_line_parser(argc, argv)
				.options(regular_options).positional(positional_options).run(), vm); // can throw

		/**
		 * --help option
		 */
		if (vm.count("help")) {
			std::cout << "Stereo workbench." << std::endl
					<< regular_options << std::endl;
			return success;
		}

		po::notify(vm); // throws on error, so do after help in case
						// there are any problems

	} catch (po::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << regular_options << std::endl;
		return error_in_command_line;
	}

	fs::path calib_file_path = fs::path(path);

	internal::print_calibration_app app(calib_file_path);

	return app.run();
}

