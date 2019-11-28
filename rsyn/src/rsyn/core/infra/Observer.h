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
 
#ifndef RSYN_DESIGN_OBSERVER_H
#define RSYN_DESIGN_OBSERVER_H

namespace Rsyn {

class DesignObserver {
friend class Rsyn::Design;
private:

	Design observedDesign;

public:
	
	// Note: The observer will not be registered to receive notifications for
	// methods that it does not overwrite. Therefore, no runtime overhead for
	// handling undesired notifications.

	virtual void
	onDesignDestruction() {}

	virtual void
	onPostInstanceCreate(Rsyn::Instance instance) {}

	virtual void
	onPreInstanceRemove(Rsyn::Instance instance) {}

	virtual void
	onPostNetCreate(Rsyn::Net net) {}

	virtual void
	onPreNetRemove(Rsyn::Net net) {}

	virtual void
	onPostCellRemap(Rsyn::Cell cell, Rsyn::LibraryCell oldLibraryCell) {}

	virtual void
	onPostPinConnect(Rsyn::Pin pin) {}

	virtual void
	onPrePinDisconnect(Rsyn::Pin pin) {}

	virtual void
	onPreInstanceMove(Rsyn::Instance) {}

	virtual void
	onPostInstanceMove(Rsyn::Instance) {}

	virtual
	~DesignObserver() {
		if (observedDesign)
			observedDesign.unregisterObserver(this);
	} // end destructor

}; // end class

} // end namespace

#endif

