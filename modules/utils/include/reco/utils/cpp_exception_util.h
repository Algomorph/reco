/*
 * cpp_exception_util.h
 *
 *     Authors: Gregory Kramida
 *     License: Apache v. 2
 *   Copyright: (c) Gregory Kramida 2015 
 */

#ifndef RECO_UTILS_CPP_EXCEPTION_UTIL_H_
#define RECO_UTILS_CPP_EXCEPTION_UTIL_H_
#pragma once

//std
#include <sstream>
#include <exception>

#define err(exception_class) (reco::utils::error_stream<exception_class>())
#define enderr (reco::utils::error_end_token::get_instance())

namespace reco {
namespace utils {

/**
 * Class that abstracts-away some of the hassle behind creating custom error messages for standard errors.
 *
 * Example usage: err(std::runtime_error) << 1 << 2 << 3 << std::endl << "hello" << " " << true << enderr;
 */
class error_end_token {
private:
	error_end_token() {
	}
	;

	//don't implement copy methods
	error_end_token(error_end_token const&) = delete;
	void operator=(error_end_token const&) = delete;
public:
	static const error_end_token& get_instance() {
		//guaranteed to be destroyed.
		//instantiated on first use.
		static error_end_token instance;
		return instance;
	}
};

template<class E>
class error_stream {
	static_assert(std::is_base_of<std::exception, E>::value,
			"Template Parameter EX_CL must implement std::exception"
	);
	private:
	std::stringstream message_stream;
	bool thrown;

public:
	error_stream() :
			message_stream(), thrown(false) {
	}

	/**
	 * Add something to the error message
	 * @param arg
	 * @return reference to self, to allow "<<" nesting
	 */
	template<typename T>
	error_stream& operator <<(const T& arg) {

		message_stream << arg;
		return *this;
	}

	/**
	 * Finally throws the error (only done on the first call to this function)
	 * @param arg error_end_token singleton
	 * @return reference to self, to allow "<<" nesting
	 */
	error_stream& operator <<(const error_end_token& arg) {
		if (!thrown) {
			throw E(message_stream.str());
			thrown = true;
		}
		return *this;
	}

	// this is the type of stringstream (basic_stringstream<char>)
	typedef std::basic_ostream<char, std::char_traits<char> > stringstream_type;

	// this is the function signature of std::endl
	typedef stringstream_type& (*standard_end_line)(stringstream_type&);

	/**
	 * Adds a newline to the error message
	 * @param manip -- usually, std::endl
	 * @return reference to self, to allow "<<" nesting
	 */
	error_stream& operator<<(standard_end_line manip) {
		// call the function, but we cannot return it's value
		manip(this->message_stream);
		message_stream << "           ";
		//throw E(message_stream.str());
		return *this;
	}

};

class not_implemented:
		public std::logic_error{
public:
	not_implemented(): std::logic_error("Function not yet implemented."){}

};

} //end namespace util
} //end namespace reco

#endif /* RECO_UTILS_CPP_EXCEPTION_UTIL_H_ */
