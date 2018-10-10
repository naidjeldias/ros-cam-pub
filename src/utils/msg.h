
// The MIT License (MIT)

// Copyright (c) 2018 gustavoengdm "Gustavo Teodoro Laureano"

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

/*
 * msg.hpp
 *
 *  Created on: Jul 14, 2018
 *      Author: gustavoengdm "Gustavo Teodoro Laureano"
 */

#ifndef _UTIL_MSG_HPP_
#define _UTIL_MSG_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#define STD_OUTPUT	stdout
#define ERROR_OUT stdout

#define DEFAULT_DECOR		"\x1B[0;38;48m"
#define NOTICE_DECOR		"\x1B[1;36;48m"
#define WARNING_DECOR		"\x1B[1;33;48m"
#define ERROR_DECOR			"\x1B[1;31;48m"
#define ERRNO_DECOR			"\x1B[1;35;48m"
#define EMPH_DECOR			DEFAULT_DECOR "\x1B[3;38;48m"

#define NOTICE_BODY( msg )	NOTICE_DECOR	"# NOTICE: "	EMPH_DECOR	"%s (line: %d, function: %s)\n" DEFAULT_DECOR msg, __FILE__, __LINE__, __func__ 
#define WARNING_BODY( msg )	WARNING_DECOR	"# WARNING: "	EMPH_DECOR	"%s (line: %d, function: %s)\n" DEFAULT_DECOR msg, __FILE__, __LINE__, __func__ 
#define ERROR_BODY( msg )	ERROR_DECOR		"# ERROR: "		EMPH_DECOR	"%s (line: %d, function: %s)\n" DEFAULT_DECOR msg, __FILE__, __LINE__, __func__ 
#define ERRNO_BODY( msg )	ERRNO_DECOR		"(SYSTEM ERROR) "	DEFAULT_DECOR	"Errno %d: %s\n" msg, errno, strerror(errno)

#define __msg( msg, ... )			{ fprintf(STD_OUTPUT, msg, ##__VA_ARGS__ ); }
#define __msg_notice( msg, ... ) 	{ fprintf(STD_OUTPUT, NOTICE_BODY(msg), ##__VA_ARGS__ ); }
#define __msg_warning( msg, ... ) 	{ fprintf(STD_OUTPUT, WARNING_BODY(msg), ##__VA_ARGS__ ); }
#define __msg_error( msg, ... ) 	{ fprintf(ERROR_OUT, ERROR_BODY(msg), ##__VA_ARGS__ ); }

#define __errno_msg	"Error errno(%d): %s", errno, strerror(errno)
#define __msg_perror( msg, ... ) 	{ fprintf(ERROR_OUT, ERRNO_BODY(msg), ##__VA_ARGS__ ); }


#define __assert_error( expr, msg, ... ) { if( (expr) ) { __msg_error( "ASSERTION(" #expr ")\n" msg, ##__VA_ARGS__); exit(0); } }

#endif /* _UTIL_MSG_HPP_ */
