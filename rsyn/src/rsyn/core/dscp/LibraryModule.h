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
 
#ifndef RSYN_LIBRARY_MODULE_DESCRIPTOR_H
#define RSYN_LIBRARY_MODULE_DESCRIPTOR_H

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <tuple>
#include <unordered_map>

namespace Rsyn {

class ModuleDescriptor {
private:
	// port names
	std::vector<std::tuple<std::string, Direction>> ports;
	
	// instance name, instance type (library cell, module)
	std::vector<std::tuple<std::string, std::string>> instances;
	
	// instance, pin, net
	std::vector<std::tuple<std::string, std::string, std::string>> connections;
public:
	
	void addPort(const std::string &name, const Direction &direction) {
		ports.push_back(std::make_tuple(name, direction));
	} // end method
	
	void addInstance(const std::string &name, const std::string &type) {
		instances.push_back(std::make_tuple(name, type));
	} // end method
	
	void addConnection(const std::string &instance, const std::string &pin, const std::string &net) {
		connections.push_back(std::make_tuple(instance, pin, net));
	} // end method
}; // end class

} // end namespace

#endif