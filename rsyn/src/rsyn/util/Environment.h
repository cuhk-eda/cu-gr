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
 
/* 
 * File:   Environment.h
 * Author: gaflach
 *
 * Created on February 25, 2016, 6:56 PM
 */

#ifndef UTIL_ENVIRONMENT_H
#define UTIL_ENVIRONMENT_H

#include <sstream>
#include <cstdlib>

class Environment {
private:
	
	template<typename T>
	static bool parse(const std::string &str, T &data) {
		std::istringstream iss( str );
		iss >> data;
		return !iss.fail();
	} // end method		
	
	template<typename T>
	static T get(const std::string &name, const T &defaultValue) {
		T data;
		const char *value = std::getenv(name.c_str());
		return value && parse(value, data)? data : defaultValue;
	} // end method
		
public:

	static int getBoolean(const std::string &name, const bool defaultValue) { return get(name, defaultValue); }
	static int getInteger(const std::string &name, const int defaultValue) { return get(name, defaultValue); }
	static float getFloat(const std::string &name, const float defaultValue) { return get(name, defaultValue); }
	static double getDouble(const std::string &name, const double defaultValue) { return get(name, defaultValue); }
	static std::string getString(const std::string &name, const std::string &defaultValue) { return get(name, defaultValue); }
	
}; // end class


#endif

