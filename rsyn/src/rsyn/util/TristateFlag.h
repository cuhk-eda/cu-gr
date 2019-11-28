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
 
#ifndef RSYN_TRISTATE_FLAG_H
#define RSYN_TRISTATE_FLAG_H

namespace Rsyn {

class TristateFlag {
private:
	bool specified : 1;
	bool flag : 1;

public:

	TristateFlag() : specified(false), flag(false) {}
	TristateFlag(const bool value) : specified(true), flag(value) {}

	bool get() const {
		return flag;
	} // end method

	void set(const bool value) {
		specified = true;
		flag = value;
	} // end method

	void clear() {
		specified = false;
		flag = false;
	} // end method

	bool isSpecified() const {
		return specified;
	} // end method

	bool isNotSpecified() const {
		return !specified;
	} // end method

	operator bool() const { 
		return get();
	} // end method
}; // end class

} // end namespace

#endif 

