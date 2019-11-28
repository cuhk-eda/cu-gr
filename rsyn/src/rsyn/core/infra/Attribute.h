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
 
#ifndef RSYN_ATTRIBUTE_H
#define RSYN_ATTRIBUTE_H

#include <deque>

namespace Rsyn {

class AttributeInitializer {
friend class Design;	
private:
	Design design;
	AttributeInitializer(Design design): design(design) {};
public:
	Design getDesign() { return design; }
}; // end class

// -----------------------------------------------------------------------------

template<typename DefaultValueType>
class AttributeInitializerWithDefaultValue {
friend class Design;	
private:
	Design design;
	DefaultValueType defaultValue;
	AttributeInitializerWithDefaultValue(Design design, DefaultValueType defaultValue)
			: design(design), defaultValue(defaultValue) {};
public:
	Design getDesign() { return design; }
	DefaultValueType &getDefaultValue() { return defaultValue; }
}; // end class

// -----------------------------------------------------------------------------

template<typename _Object, typename _ObjectExtension>
class AttributeImplementation;

// -----------------------------------------------------------------------------

template <typename RsynObject, typename RsynObjectExtension>
class Attribute : public std::shared_ptr<AttributeImplementation<RsynObject, RsynObjectExtension>> {
public:	
	Attribute() {}
	
	Attribute(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer);
	
	template<typename DefaultValueType>
	Attribute(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer);
	
	RsynObjectExtension &operator[](RsynObject obj);
	const RsynObjectExtension &operator[](RsynObject obj) const;
}; // end class

// -----------------------------------------------------------------------------

template<typename _Object, typename _ObjectReference, typename _ObjectExtension>
class AttributeBase {
private:
	// [TODO] Make design and list const.
	
	Design clsDesign;
	List<_Object> *clsListPtr;
	_ObjectExtension clsDefaultValue;
	
	typename List<_Object>::CreateElementCallbackHandler clsHandlerOnCreate;
	typename List<_Object>::DestructorCallbackHandler clsListDestructorCallbackHandler;
	
	std::deque<_ObjectExtension> clsData;
	
	void accommodate(const Index index) {
		if (index >= clsData.size()) {
			clsData.resize(index + 1, clsDefaultValue);
		} // end if
	} // end method
	
protected:
	
	void setupCallbacks() {
		clsHandlerOnCreate = clsListPtr->addCreateCallback([&](const int index) {
			accommodate(clsListPtr->largestId());
		}); // end method
		
		clsListDestructorCallbackHandler = clsListPtr->addDestructorEventCallback([&]() {
			clsListPtr = nullptr;
		}); // end method		
		
	} // end method
	
	void load(Design design, List<_Object> &list, _ObjectExtension defaultValue = _ObjectExtension()) {
		clsDesign = design;
		clsListPtr = &list;
		clsDefaultValue = defaultValue;
		accommodate(clsListPtr->largestId());
		setupCallbacks();
	} // end method		
	
public:
	
	AttributeBase() : clsDesign(nullptr), clsListPtr(nullptr) {
	} // end constructor
	
	AttributeBase(const AttributeBase<_Object, _ObjectReference, _ObjectExtension> &other) {
		operator=(other);
	} // end constructor
	
	AttributeBase(Design design, List<_Object> &list) {
		load(design, list);
	} // end constructor

	AttributeBase<_Object, _ObjectReference, _ObjectExtension> &
	operator=(const AttributeBase<_Object, _ObjectReference, _ObjectExtension> &other) {
		unload();
		
		clsDesign = other.clsDesign;
		clsListPtr = other.clsListPtr;
		clsData = other.clsData;
						
		// When a layer gets copied, we need to setup new callbacks.
		setupCallbacks();
		
		return *this;
	} // end method
	
	~AttributeBase() {
		unload();
	} // end constructor
	
	void unload() {
		if (clsDesign) {
			clsDesign = nullptr;
		} // end if
		
		if (clsListPtr) {
			clsListPtr->deleteCreateCallback(clsHandlerOnCreate);
			clsListPtr->deleteDestructorCallback(clsListDestructorCallbackHandler);
			clsListPtr = nullptr;
		} // end if

		clsData.clear();
		clsData.shrink_to_fit();
	} // end method	

	inline _ObjectExtension &operator[](_ObjectReference obj) { return clsData[clsDesign.getId(obj)]; }
	inline const _ObjectExtension &operator[](_ObjectReference obj) const { return clsData[clsDesign.getId(obj)]; }

}; // end class	

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: (Generic) - Internal Use Only
////////////////////////////////////////////////////////////////////////////////

template<typename _Object, typename _ObjectExtension>
class AttributeImplementation {
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: Net
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::Net, _ObjectExtension> 
	: public Rsyn::AttributeBase<NetData, Net, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<NetData, Net, _ObjectExtension>::load(design, design.data->nets);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<NetData, Net, _ObjectExtension>::load(design, design.data->nets, initializer.getDefaultValue());
	} // end operator		
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: Instance
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::Instance, _ObjectExtension> 
	: public Rsyn::AttributeBase<InstanceData, Instance, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<InstanceData, Instance, _ObjectExtension>::load(design, design.data->instances);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<InstanceData, Instance, _ObjectExtension>::load(design, design.data->instances, initializer.getDefaultValue());
	} // end operator	
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: Pin
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::Pin, _ObjectExtension> 
	: public Rsyn::AttributeBase<PinData, Pin, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<PinData, Pin, _ObjectExtension>::load(design, design.data->pins);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<PinData, Pin, _ObjectExtension>::load(design, design.data->pins, initializer.getDefaultValue());
	} // end operator		
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: Arc
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::Arc, _ObjectExtension> 
	: public Rsyn::AttributeBase<ArcData, Arc, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<ArcData, Arc, _ObjectExtension>::load(design, design.data->arcs);
	} // end operator

	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<ArcData, Arc, _ObjectExtension>::load(design, design.data->arcs, initializer.getDefaultValue());
	} // end operator
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: LibraryCell
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::LibraryCell, _ObjectExtension> 
	: public Rsyn::AttributeBase<LibraryCellData, LibraryCell, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryCellData, LibraryCell, _ObjectExtension>::load(design, design.data->libraryCells);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }	
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryCellData, LibraryCell, _ObjectExtension>::load(design, design.data->libraryCells, initializer.getDefaultValue());
	} // end operator	
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: LibraryPin
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::LibraryPin, _ObjectExtension> 
	: public Rsyn::AttributeBase<LibraryPinData, LibraryPin, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryPinData, LibraryPin, _ObjectExtension>::load(design, design.data->libraryPins);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }	
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryPinData, LibraryPin, _ObjectExtension>::load(design, design.data->libraryPins, initializer.getDefaultValue());
	} // end operator		
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute Layer: LibraryArc
////////////////////////////////////////////////////////////////////////////////

template<typename _ObjectExtension>
class AttributeImplementation<Rsyn::LibraryArc, _ObjectExtension> 
	: public Rsyn::AttributeBase<LibraryArcData, LibraryArc, _ObjectExtension> {
public:
	AttributeImplementation() {}
	
	AttributeImplementation(AttributeInitializer initializer) { operator=(initializer); }
	void operator=(AttributeInitializer initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryArcData, LibraryArc, _ObjectExtension>::load(design, design.data->libraryArcs);
	} // end operator
	
	template<typename DefaultValueType>
	AttributeImplementation(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) { operator=(initializer); }	
	template<typename DefaultValueType>
	void operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
		Design design = initializer.getDesign();
		AttributeBase<LibraryArcData, LibraryArc, _ObjectExtension>::load(design, design.data->libraryArcs, initializer.getDefaultValue());
	} // end operator		
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Attribute
////////////////////////////////////////////////////////////////////////////////

template<typename RsynObject, typename RsynObjectExtension>
inline
void Attribute<RsynObject, RsynObjectExtension>::operator=(AttributeInitializer initializer) {
	this->reset(new AttributeImplementation<RsynObject, RsynObjectExtension>(initializer));
} // end method

template<typename RsynObject, typename RsynObjectExtension>
template<typename DefaultValueType>
inline
void Attribute<RsynObject, RsynObjectExtension>::operator=(AttributeInitializerWithDefaultValue<DefaultValueType> initializer) {
	this->reset(new AttributeImplementation<RsynObject, RsynObjectExtension>(initializer));
} // end method

template<typename RsynObject, typename RsynObjectExtension>
inline
RsynObjectExtension &Attribute<RsynObject, RsynObjectExtension>::operator[](RsynObject obj) {
	return (*this)->operator[](obj); 
} // end method

template<typename RsynObject, typename RsynObjectExtension>
inline
const RsynObjectExtension &Attribute<RsynObject, RsynObjectExtension>::operator[](RsynObject obj) const {
	return (*this)->operator[](obj); 
} // end method

} // end namespace


#endif

