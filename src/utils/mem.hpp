

#ifndef _UTIL_MEM_HPP_
#define _UTIL_MEM_HPP_

namespace util {
	namespace mem {

		template<typename T> T& rw(const T & x) {
			return (T&) x;
		}

		template<typename T> T*& rwp(T* const & x) {
			return (T* &) x;
		}
	}
}

#endif // _UTIL_MEM_HPP_