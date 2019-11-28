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
 
#ifndef RSYN_PHYSICALDESIGN_ATTRIBUTE_H
#define RSYN_PHYSICALDESIGN_ATTRIBUTE_H

#include <deque>

namespace Rsyn {

class PhysicalAttributeInitializer {
	friend class PhysicalDesign;
private:
	PhysicalDesign design;

	PhysicalAttributeInitializer(PhysicalDesign design) : design(design) {
	};
public:

	PhysicalDesign getPhysicalDesign() {
		return design;
	} // end method 
}; // end class

// -----------------------------------------------------------------------------

template<typename PhysicalDefaultValueType>
class PhysicalAttributeInitializerWithDefaultValue {
	friend class PhysicalDesign;
private:
	PhysicalDesign design;
	PhysicalDefaultValueType defaultValue;

	PhysicalAttributeInitializerWithDefaultValue(PhysicalDesign design, PhysicalDefaultValueType defaultValue)
	: design(design), defaultValue(defaultValue) {
	};
public:

	PhysicalDesign getPhysicalDesign() {
		return design;
	}

	PhysicalDefaultValueType &getDefaultValue() {
		return defaultValue;
	}
}; // end class

// -----------------------------------------------------------------------------

template<typename _PhysicalObject, typename _PhysicalObjectExtension>
class PhysicalAttributeImplementation;

// -----------------------------------------------------------------------------

template <typename RsynPhysicalObject, typename RsynPhysicalObjectExtension>
class PhysicalAttribute : public std::shared_ptr<PhysicalAttributeImplementation<RsynPhysicalObject, RsynPhysicalObjectExtension>>
{
	public:

	PhysicalAttribute() {
	}

	PhysicalAttribute(PhysicalAttributeInitializer initializer) {
		operator=(initializer);
	}
	void operator=(PhysicalAttributeInitializer initializer);

	template<typename PhysicalDefaultValueType >
		PhysicalAttribute(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		operator=(initializer);
	}
	template<typename PhysicalDefaultValueType>
		void operator=(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer);

	RsynPhysicalObjectExtension &operator[](RsynPhysicalObject obj);
	const RsynPhysicalObjectExtension &operator[](RsynPhysicalObject obj) const;
}; // end class

// -----------------------------------------------------------------------------

template<typename _PhysicalObject, typename _PhysicalObjectReference, typename _PhysicalObjectExtension>
class PhysicalAttributeBase {
private:
	// [TODO] Make design and list const.

	PhysicalDesign clsPhysicalDesign;
	List<_PhysicalObject> *clsListPtr;
	_PhysicalObjectExtension clsDefaultValue;

	typename List<_PhysicalObject>::CreateElementCallbackHandler clsHandlerOnCreate;
	typename List<_PhysicalObject>::DestructorCallbackHandler clsListDestructorCallbackHandler;

	std::deque<_PhysicalObjectExtension> clsData;

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

	void load(PhysicalDesign design, List<_PhysicalObject> &list, _PhysicalObjectExtension defaultValue = _PhysicalObjectExtension()) {
		clsPhysicalDesign = design;
		clsListPtr = &list;
		clsDefaultValue = defaultValue;
		accommodate(clsListPtr->largestId());
		setupCallbacks();
	} // end method		

public:

	PhysicalAttributeBase() : clsPhysicalDesign(nullptr), clsListPtr(nullptr) {
	} // end constructor

	PhysicalAttributeBase(const PhysicalAttributeBase<_PhysicalObject, _PhysicalObjectReference, _PhysicalObjectExtension> &other) {
		operator=(other);
	} // end constructor

	PhysicalAttributeBase(PhysicalDesign design, List<_PhysicalObject> &list) {
		load(design, list);
	} // end constructor

	PhysicalAttributeBase<_PhysicalObject, _PhysicalObjectReference, _PhysicalObjectExtension> &
		operator=(const PhysicalAttributeBase<_PhysicalObject, _PhysicalObjectReference, _PhysicalObjectExtension> &other) {
		unload();

		clsPhysicalDesign = other.clsPhysicalDesign;
		clsListPtr = other.clsListPtr;
		clsData = other.clsData;

		// When a layer gets copied, we need to setup new callbacks.
		setupCallbacks();

		return *this;
	} // end method

	~PhysicalAttributeBase() {
		unload();
	} // end constructor

	void unload() {
		if (clsPhysicalDesign) {
			clsPhysicalDesign = nullptr;
		} // end if

		if (clsListPtr) {
			clsListPtr->deleteCreateCallback(clsHandlerOnCreate);
			clsListPtr->deleteDestructorCallback(clsListDestructorCallbackHandler);
			clsListPtr = nullptr;
		} // end if

		clsData.clear();
		clsData.shrink_to_fit();
	} // end method	

	inline _PhysicalObjectExtension &operator[](_PhysicalObjectReference obj) {
		return clsData[clsPhysicalDesign.getId(obj)];
	}

	inline const _PhysicalObjectExtension &operator[](_PhysicalObjectReference obj) const {
		return clsData[clsPhysicalDesign.getId(obj)];
	}

}; // end class	

////////////////////////////////////////////////////////////////////////////////
// PhysicalAttribute Layer: (Generic) - Internal Use Only
////////////////////////////////////////////////////////////////////////////////

template<typename _PhysicalObject, typename _PhysicalObjectExtension>
class PhysicalAttributeImplementation {
}; // end class

////////////////////////////////////////////////////////////////////////////////
// PhysicalAttribute Layer: PhysicalRow
////////////////////////////////////////////////////////////////////////////////

template<typename _PhysicalObjectExtension>
class PhysicalAttributeImplementation<Rsyn::PhysicalRow, _PhysicalObjectExtension>
: public Rsyn::PhysicalAttributeBase<PhysicalRowData, PhysicalRow, _PhysicalObjectExtension> {
public:

	PhysicalAttributeImplementation() {
	}

	PhysicalAttributeImplementation(PhysicalAttributeInitializer initializer) {
		operator=(initializer);
	}

	void operator=(PhysicalAttributeInitializer initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalRowData, PhysicalRow, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalRows);
	} // end operator

	template<typename PhysicalDefaultValueType>
	PhysicalAttributeImplementation(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		operator=(initializer);
	}

	template<typename PhysicalDefaultValueType>
	void operator=(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalRowData, PhysicalRow, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalRows, initializer.getDefaultValue());
	} // end operator		
}; // end class


////////////////////////////////////////////////////////////////////////////////
// PhysicalAttribute Layer: PhysicalLayer
////////////////////////////////////////////////////////////////////////////////

template<typename _PhysicalObjectExtension>
class PhysicalAttributeImplementation<Rsyn::PhysicalLayer, _PhysicalObjectExtension>
: public Rsyn::PhysicalAttributeBase<PhysicalLayerData, PhysicalLayer, _PhysicalObjectExtension> {
public:

	PhysicalAttributeImplementation() {
	}

	PhysicalAttributeImplementation(PhysicalAttributeInitializer initializer) {
		operator=(initializer);
	}

	void operator=(PhysicalAttributeInitializer initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalLayerData, PhysicalLayer, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalLayers);
	} // end operator

	template<typename PhysicalDefaultValueType>
	PhysicalAttributeImplementation(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		operator=(initializer);
	}

	template<typename PhysicalDefaultValueType>
	void operator=(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalLayerData, PhysicalLayer, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalLayers, initializer.getDefaultValue());
	} // end operator		
}; // end class


////////////////////////////////////////////////////////////////////////////////
// PhysicalAttribute Layer: PhysicalSpacing
////////////////////////////////////////////////////////////////////////////////

template<typename _PhysicalObjectExtension>
class PhysicalAttributeImplementation<Rsyn::PhysicalSpacing, _PhysicalObjectExtension>
: public Rsyn::PhysicalAttributeBase<PhysicalSpacingData, PhysicalSpacing, _PhysicalObjectExtension> {
public:

	PhysicalAttributeImplementation() {
	}

	PhysicalAttributeImplementation(PhysicalAttributeInitializer initializer) {
		operator=(initializer);
	}

	void operator=(PhysicalAttributeInitializer initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalSpacingData, PhysicalSpacing, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalSpacing);
	} // end operator

	template<typename PhysicalDefaultValueType>
	PhysicalAttributeImplementation(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		operator=(initializer);
	}

	template<typename PhysicalDefaultValueType>
	void operator=(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
		PhysicalDesign design = initializer.getPhysicalDesign();
		PhysicalAttributeBase<PhysicalSpacingData, PhysicalSpacing, _PhysicalObjectExtension>::load(design, design.data->clsPhysicalSpacing, initializer.getDefaultValue());
	} // end operator		
}; // end class


////////////////////////////////////////////////////////////////////////////////
// PhysicalAttribute
////////////////////////////////////////////////////////////////////////////////

template<typename RsynPhysicalObject, typename RsynPhysicalObjectExtension>
inline
void PhysicalAttribute<RsynPhysicalObject, RsynPhysicalObjectExtension>::operator=(PhysicalAttributeInitializer initializer) {
	this->reset(new PhysicalAttributeImplementation<RsynPhysicalObject, RsynPhysicalObjectExtension>(initializer));
} // end method

template<typename RsynPhysicalObject, typename RsynPhysicalObjectExtension>
template<typename PhysicalDefaultValueType>
inline
void PhysicalAttribute<RsynPhysicalObject, RsynPhysicalObjectExtension>::operator=(PhysicalAttributeInitializerWithDefaultValue<PhysicalDefaultValueType> initializer) {
	this->reset(new PhysicalAttributeImplementation<RsynPhysicalObject, RsynPhysicalObjectExtension>(initializer));
} // end method

template<typename RsynPhysicalObject, typename RsynPhysicalObjectExtension>
inline
RsynPhysicalObjectExtension &PhysicalAttribute<RsynPhysicalObject, RsynPhysicalObjectExtension>::operator[](RsynPhysicalObject obj) {
	return (*this)->operator[](obj);
} // end method

template<typename RsynPhysicalObject, typename RsynPhysicalObjectExtension>
inline
const RsynPhysicalObjectExtension &PhysicalAttribute<RsynPhysicalObject, RsynPhysicalObjectExtension>::operator[](RsynPhysicalObject obj) const {
	return (*this)->operator[](obj);
} // end method

} // end namespace


#endif

