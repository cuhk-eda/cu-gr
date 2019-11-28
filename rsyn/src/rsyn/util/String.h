/* Copyright 2014-2018 Rsyn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#ifndef UTIL_STRING_H
#define	UTIL_STRING_H

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>

class String {
public:
	
	template< typename T >
	static bool parse(const std::string &str, T &data) {
		std::istringstream iss( str );
		iss >> data;
		return !iss.fail();
	} // end function	

	static bool parse(const std::string &str, std::string &data) { 
		return (data=str, true); 
	} // end function
	
	static void printCentered(std::ostream &out, const std::string &text, const int width = 80) {
		// DOES NOT WORKS WHEN TEXT HAS \t
		const int start = (width - text.size()) / 2;
		if ( start > 0 )
			out << std::setw(start) << std::setfill( ' ' ) << ' ';
		out << text;
	} // end function		
	
}; // end class

#endif

