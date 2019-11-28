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
 
#ifndef RSYN_EXCEPTION_H
#define RSYN_EXCEPTION_H

#include <string>
#include <exception>

namespace Rsyn {

class Exception : public std::exception {
public:
	Exception(const std::string &msg) : std::exception(),  msg(msg) {}
	virtual const char* what() const throw() { return msg.c_str(); };
private:
	std::string msg;
}; // end class

// -----------------------------------------------------------------------------

class CellAlreadyExistsException : public Exception {
public:
	CellAlreadyExistsException(const std::string &name) :
			Exception("Cell '" + name + "' already exists.") {}
}; // end class

// -----------------------------------------------------------------------------

class CellNotFoundException : public Exception {
public:
	CellNotFoundException(const std::string &name) :
			Exception("Cell '" + name + "' not found.") {}
}; // end class

// -----------------------------------------------------------------------------

class LibraryCellAlreadyExistsException : public Exception {
public:
	LibraryCellAlreadyExistsException(const std::string &name) :
			Exception("Library cell '" + name + "' already exists.") {}
}; // end class

// -----------------------------------------------------------------------------

class LibraryCellNotFoundException : public Exception {
public:
	LibraryCellNotFoundException(const std::string &libCellName) :
			Exception("Library cell '" + libCellName + "' not found.") {}
}; // end class

// -----------------------------------------------------------------------------

class LibraryPinNotFoundException : public Exception {
public:
	LibraryPinNotFoundException(const std::string &libCellName, const std::string &libPinName) :
			Exception("Library pin '" + libPinName + "' not found in library cell '" + libCellName + "'.") {}
}; // end class

// -----------------------------------------------------------------------------

class NetAlreadyExistsException : public Exception {
public:
	NetAlreadyExistsException(const std::string &name) :
			Exception("Net '" + name + "' already exists.") {}
}; // end class

// -----------------------------------------------------------------------------

class PinNotFoundException : public Exception {
public:
	PinNotFoundException(const std::string &libCellName, const std::string &cellName, const std::string &pinName) :
			Exception("Pin '" + pinName + "' not found in cell '" + cellName + "'(" + libCellName + ").") {}
}; // end class

// -----------------------------------------------------------------------------

class IncompatibleLibraryCellForRemapping : public Exception {
public:
	IncompatibleLibraryCellForRemapping(const std::string &oldLibraryCell, const std::string &newLibraryCell) :
			Exception("Library cell '" + newLibraryCell + "' is not compatible to library cell '" + oldLibraryCell + "'.") {}
}; // end class

} // end namespace

#endif

