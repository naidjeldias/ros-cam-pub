
#ifndef _MCORE_IMPL_BAOP_HPP_
#define _MCORE_IMPL_BAOP_HPP_


namespace mcore {
    namespace impl {

        template <typename T1, typename T2>
        void arrayset( T1 * dst, int sz_dst, int st_dst, T2 v ) 
        {
            int i;
            for( i = 0; i < sz_dst; i++ )
            {
                *dst = v;
                dst += st_dst;
            }
        }

        template <typename T1, typename T2>
        void arrayset2d( T1 * dst, int nrows, int ncols, int depth, int st_dst, T2 v ) 
        {
            int i, j;
            int inc = st_dst - ncols;
            for( i = 0; i < nrows; i++) {
                for( j = 0; j < ncols; j++ ) {
                    *dst = v;
                    dst++;
                }
                dst += inc;
            }
        }

        template <typename T1, typename T2>
        void arrayset3d( T1 * dst, int nrows, int ncols, int depth, int st_dst, T2 v ) 
        {
            int i, j, k;
            int inc_dst;

            inc_dst = (st_dst - ncols*depth);
        
            for( i = 0; i < nrows; i++) {
                for( j = 0; j < ncols; j++ ) {
                    for( k =0; k < depth; k++ ) {
                        *dst = v;
                        dst++;
                    }
                }
                dst += inc_dst;
            }
        }

        template <typename T1, typename T2>
        void arraycpy( T1 * dst, int st_dst, T2 * orig, int sz_orig, int st_orig ) 
        {
            int i;
            for( i = 0; i < sz_orig; i++ )
            {
                *dst = *orig;
                dst += st_dst;
                orig += st_orig;
            }
        }

        template <typename T1, typename T2>
        void arraycpy2d( T1 * dst, int st_dst, T2 * orig, int nrows, int ncols, int st_orig ) 
        {
            int i, j;
            int inc_dst = st_dst - ncols;
            int inc_orig = st_orig - ncols;

            for( i = 0; i < nrows; i++) {
                for( j = 0; j < ncols; j++ ) {
                    *dst = *orig;
                    dst++;
                    orig++;
                }
                dst += inc_dst;
                orig += inc_orig;
            }
        }

        template <typename T1, typename T2>
        void arraycpy3d( T1 * dst, int st_dst, T2 * orig, int nrows, int ncols, int depth, int st_orig ) 
        {
            int i, j, k;
            int inc_dst, inc_orig;

            inc_dst = (st_dst - ncols*depth );
            inc_orig = (st_orig - ncols*depth );
        
            for( i = 0; i < nrows; i++) {
                for( j = 0; j < ncols; j++ ) {
                    for( k =0; k < depth; k++ ) {
                        *dst = *orig;
                        dst++;
                        orig++;
                    }
                }
                dst += inc_dst;
                orig += inc_orig;
            }
        }
    }
}


#endif // _MCORE_IMPL_BAOP_HPP_