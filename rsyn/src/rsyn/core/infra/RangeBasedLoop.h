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
 
#ifndef UTIL_RANGEBASEDLOOP_H
#define	UTIL_RANGEBASEDLOOP_H

////////////////////////////////////////////////////////////////////////////////
// Author: Guilherme Flach
// Date: 2015-02-17
//
// Description
// -----------
// This class makes it easier to create range-based loops.
//
// How To Use It
// -------------
// Create a class/struct with the following methods:
//     1) Object current() {...}
//     2) void next() {...}
//     3) bool filter() {...}
//     4) bool stop() {...}
//
// which the following meaning
//
//     1) returns the current object in the collection;
//     2) moves to the next object in the collection;
//     3) indicates if the current object should be filtered (i.e. not
//        processed), this class will call next() until filter() returns true
//        or the end of collection is reached;
//     4) indicates if we reached the end of the collection.
//
// Example
// -------
// Traverse only non-zero elements of a vector (i.e. filter out zeros):
//
//    // This is the class the user (you) needs to provide to create a
//    // ranged-based loop.
//    class NonZeroElementsCollection {
//    private:
//        const std::vector<int> &v;
//        std::size_t index;
//    public:
//        NonZeroElementsCollection(const std::vector<int> &v) 
//            : v(v), index(0) {}
//
//        int current() {return v[index];}
//        void next() {index++;}
//        bool filter() {return current() == 0;}
//        bool stop() {return index >= v.size();}        
//    }; // end class
//
//    // Give it a nice name...
//    typedef Range<NonZeroElementsCollection> allNonZeroElements;
//
//    // Testing...
//    int main() {
//        std::vector<int> v = {0, 1, 0, 2, 0, 3, 4, 0, 0, 0, 5};
//
//        for (int value : allNonZeroElements(v)) {
//            std::cout << value << " ";
//        } // end for
//        std::cout << "\n";
//
//        return 1;
//    } // end function
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// According to the C++11 standard, the following range-loop statement
// 
//        for ( declaration : expression ) statement
//
// is equivalent to the the following statament
//
//        {
//            auto&& __range = expression;
//            for (auto __begin = begin-expression,
//                      __end = end-expression;
//                 __begin != __end;
//                 ++__begin
//            ) {
//                declaration = *__begin;
//                statement
//            }
//        }
//
// So that, we can override the operator != of an iterator to make it
// return false whenever the stop condition was reached in the begin 
// iterator in code above.
//
// Source: http://en.cppreference.com/w/cpp/language/range-for
////////////////////////////////////////////////////////////////////////////////

template<class Collection>
class Range {
private:
	Collection collection;

	struct RangeIterator {
		Collection *collection;

		RangeIterator() : collection(nullptr) {}
		RangeIterator(Collection &collection_) : collection(&collection_) {
			while (!collection->stop() && collection->filter()) { // stop must be first
				collection->next();
			} // end while
		} // end constructor

		void operator++() { 
			do {
				collection->next();
			} while (!collection->stop() && collection->filter()); // stop must be first
		} // end method

		bool operator!=(const RangeIterator &) {
			return !collection->stop();
		} // end method

		auto operator*() -> decltype(collection->current()) {
			return collection->current();
		} // end method	

	}; // end class

public:

	Range(Collection &&collection) : collection(collection) {}

	RangeIterator begin() { return RangeIterator(collection); }
	RangeIterator end() { return RangeIterator(); /*dummy, not used */}
}; // end class	

////////////////////////////////////////////////////////////////////////////////
// Traverse a collection backwards.
////////////////////////////////////////////////////////////////////////////////

template <typename Collection>
class BackwardsCollection {
private:
    const Collection &collection;
public:
    explicit BackwardsCollection(const Collection &t) : collection(t) {}
    typename Collection::const_reverse_iterator begin() const { return collection.rbegin(); }
    typename Collection::const_reverse_iterator end()   const { return collection.rend(); }
}; // end class

////////////////////////////////////////////////////////////////////////////////
// Returns an specific element of a tuple collection.
////////////////////////////////////////////////////////////////////////////////

template<std::size_t index, typename... _Elements>
class TupleElement : public std::tuple<_Elements...> {
public:
	typedef typename std::tuple_element<index, std::tuple<_Elements...>>::type T;
	
	TupleElement() {}
	TupleElement(const std::tuple<_Elements...> &t) : std::tuple<_Elements...>(t) {}
	void operator=(const std::tuple<_Elements...> &t) { std::tuple<_Elements...>::operator=(t); }
	operator T() { return std::get<index>(*this);} 
}; // end class

#endif

