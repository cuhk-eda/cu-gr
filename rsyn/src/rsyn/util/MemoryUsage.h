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
 
#ifndef MEMORY_USAGE_H
#define	MEMORY_USAGE_H

#ifdef __linux__
#include <sys/time.h>
#include <sys/resource.h>

class MemoryUsage {
public:	
	// Retrieve memory usage in MB.
	static int getMemoryUsage() {
		struct rusage usage; 
		int ret; 
		ret = getrusage(RUSAGE_SELF, &usage);
		return usage.ru_maxrss / 1000;		
	} // end method
}; // end class

#else

class MemoryUsage {
public:
	static int getMemoryUsage() { return 0; }
}; // end class

#endif

#endif

