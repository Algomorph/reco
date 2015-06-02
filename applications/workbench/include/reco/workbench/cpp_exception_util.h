/*
 * cpp_exception_util.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_CPP_EXCEPTION_UTIL_H_
#define RECO_CPP_EXCEPTION_UTIL_H_
#pragma once

//std
#include <sstream>
#include <exception>

namespace reco {

//TODO: make utility module and put this there

/**
 * Class that abstracts-away some of the hassle behind creating custom error messages for standard errors.
 *
 * Example usage: err<std::runtime_error>() << 3 << " hedgehogs " << "are" << " cute: " << true << std::endl;
 */

template<class E>
class err {
	static_assert(std::is_base_of<std::exception, E>::value,
			"Template Parameter EX_CL must implement std::exception"
	);
private:
	std::stringstream message_stream;

public:
	err():message_stream(){
		std::cout << "Got to constructor!" << std::endl;
	}

	template<typename T>
	err& operator <<(const T& arg) {
		std::cout << "Got to input!" << std::endl;
		message_stream << arg;
		return *this;
	}

	// this is the type of stringstream (basic_stringstream<char>)
	typedef std::basic_ostream<char, std::char_traits<char> > stringstream_type;

	// this is the function signature of std::endl
	typedef stringstream_type& (*standard_end_line)(stringstream_type&);

	err& operator<<(standard_end_line manip){
	        // call the function, but we cannot return it's value
			manip(this->message_stream);
	        throw E(message_stream.str());
	        return *this;
	}


};

} //end namespace reco

#endif /* RECO_WORKBENCH_CPP_EXCEPTION_UTIL_H_ */
