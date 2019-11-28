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
 
#ifndef RSYN_SERVICE_H
#define RSYN_SERVICE_H

#include "rsyn/util/Json.h"

namespace Rsyn {

class Session;


enum ServiceRequestType {
	SERVICE_OPTIONAL,
	SERVICE_MANDATORY
}; // end enum

class Service {
public:
	virtual void start(const Rsyn::Json &params) = 0;
	virtual void stop() = 0;
}; // end class

} // end namespace


#endif

