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

#ifndef RSYN_DATABASE_H
#define RSYN_DATABASE_H

#include <string>

#include <Serializable.h>
#include <SerializationStream.h>

namespace Rsyn {
class Database {
public:

	void read(const std::string &key, Serializable &data);
	void write(const std::string &key, const Serializable &data);

};
} // end namespace

#endif 

