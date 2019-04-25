/*
 * func_obj.hpp
 *
 *  Created on: Oct 31, 2017
 *      Author: Gustavo Teodoro Laureno, gustavo
 */

#ifndef _MCORE_FUNC_OBJ_HPP_
#define _MCORE_FUNC_OBJ_HPP_

namespace mcore {

	template<typename Derived> class FuncObj {
		public:
			FuncObj() {
			}
			~FuncObj() {
			}

			inline const Derived& get_derived() const {
				return (const Derived &) *this;
			}

	};
} // namespace core

#endif /* _MCORE_FUNC_OBJ_HPP_ */
