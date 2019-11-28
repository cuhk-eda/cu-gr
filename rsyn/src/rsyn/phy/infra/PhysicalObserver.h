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
 
#ifndef RSYN_PHYSICAL_OBSERVER_H
#define RSYN_PHYSICAL_OBSERVER_H

namespace Rsyn {

class PhysicalDesignObserver {
friend class Rsyn::PhysicalDesign;
private:
	PhysicalDesign clsPhDesign;

public:
	
	// Note: The observer will not be registered to receive notifications for
	// methods that it does not overwrite. Therefore, no runtime overhead for
	// handling undesired notifications.

	virtual void
	onPhysicalDesignDestruction() {}

	virtual void
	onPostNetRoutingChange(Rsyn::PhysicalNet physicalNet) {}
	
	virtual
	~PhysicalDesignObserver() {
		if (clsPhDesign)
			clsPhDesign.unregisterObserver(this);
	} // end destructor

}; // end class

} // end namespace


#endif /* RSYN_PHYSICALOBSERVER_H */

