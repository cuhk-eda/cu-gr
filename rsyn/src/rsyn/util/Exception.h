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
 
#ifndef EXCEPTION_H
#define	EXCEPTION_H

#include <exception>
#include <string>
#include <ostream>

class Exception: public std::exception {
	friend std::ostream &operator<<(std::ostream &out, const Exception &e) { 
		return out << e.what();
	} // end method	
	
public:

	explicit Exception(const std::string& message) :  clsMsg(message) {}
	virtual ~Exception() throw (){}

	virtual const char* what() const throw (){
       return clsMsg.c_str();
    } // end method
	
protected:
    std::string clsMsg;
}; // end class

#endif

