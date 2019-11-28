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
 
#ifndef RSYN_LIST_H
#define RSYN_LIST_H

#include <cmath>
#include <deque>
#include <list>
#include <vector>
#include <functional>
#include <iostream>

namespace Rsyn {

template<typename T> 
struct Element {
	T value;
    bool deleted;

    Element() : deleted(true) {};
	Element(T value) : value(value), deleted(false) {};
}; // end struct 

//--------------------------------------------------------------------------

template<typename T> struct ElementPointer {
private:
	// For some reason the compiler allows ElementPointer to be silently casted
	// to int, which led to hard to find bugs. Now we explicitly tell the 
	// compiler to not allow such conversion by making the cast operator 
	// private.
	operator int() const { return 0; }
protected:
	Element<T> *e;

public:	
	T &getValue() { return e->value; }	
	Element<T> *getPointer() { return e; }

	ElementPointer() : e( nullptr ) {};	
	ElementPointer(Element<T> *e) : e(e) {};

	T * operator->() { return &(e->value); };
	const T * operator->() const { return &(e->value); };
	
	bool operator!() const { return e == nullptr; };
	bool operator==(const ElementPointer<T> &p) const { return e == p.e; };
	bool operator!=(const ElementPointer<T> &p) const { return e != p.e; };
	
	operator bool() const { return e; }
	
	// Used in map-like data structures.
	friend bool operator<(const ElementPointer<T> &left, const ElementPointer<T> &right) {
		// [IMPORTANT] We don't use the pointer (e) directly to avoid 
		// non-determinism behavior. Note that the pointer address may change
		// from execution to execution and hence the mapping function may return
		// elements in different order leading to different results.
		return left.e->value.getId() < right.e->value.getId();
	} // end method
}; // end struct 

//--------------------------------------------------------------------------

template<typename T, unsigned int DEFAULT_CHUNK_SIZE> 
struct Chunk {		
	Element<T> elements[DEFAULT_CHUNK_SIZE];
}; // end struct

//--------------------------------------------------------------------------

//
// DEFAULT CHUNCK SIZE
// 
// Date: 2016-03-02
//
// superblu18
//    10 runtime: 126.876 s memory: +2804 MB
//   100 runtime: 113.372 s memory: +2790 MB
//  1000 runtime: 112.172 s memory: +2786 MB
// 10000 runtime: 114.887 s memory: +2790 MB
//
// superblue10
//    10 runtime: 331.377 s memory: +6293 MB
//   100 runtime: 326.238 s memory: +6258 MB
//  1000 runtime: 324.864 s memory: +6258 MB
// 10000 runtime: 324.034 s memory: +6259 MB
//

template<typename T, unsigned int DEFAULT_CHUNK_SIZE = 1000>
class List {
public:	
	
	typedef std::function<void(const int index)> CreateElementCallback;
	typedef std::function<void(const int index)> RemoveElementCallback;
	typedef std::function<void()> DestructorCallback;
	
	typedef std::list<CreateElementCallback>::iterator CreateElementCallbackHandler;
	typedef std::list<RemoveElementCallback>::iterator RemoveElementCallbackHandler;
	typedef std::list<DestructorCallback>::iterator DestructorCallbackHandler;
	
private:	
	// Iterator validity (std::deque)
	// ------------------------------
	// In case the container shrinks, all iterators, pointers and references to 
	// elements that have not been removed remain valid after the resize and refer
	// to the same elements they were referring to before the call.
	// If the container expands, all iterators are invalidated, but existing
	// pointers and references remain valid, referring to the same elements they 
	// were referring to before.
	
	std::deque<Chunk<T, DEFAULT_CHUNK_SIZE>> chunks;
	std::list<int> available;
	std::list<CreateElementCallback> callbackOnCreate;
	std::list<RemoveElementCallback> callbackOnRemove;
	std::list<DestructorCallback> callbackOnDestructor;

	int numElements;
	int lastInsertedElementId;
	int currentChunk;
	int currentChunkFreeSpace;

	Element<T> *create_internal() {
		Element<T> *e = nullptr;

		const int recycle = recycleId();
		
		if (recycle != -1) {
			// Recycle a previously removed element...
			e = get(recycle);
			available.pop_back();
			
			// Clean-up the recycled element.
			*e = T();
			lastInsertedElementId = recycle;
		} else {
			if (currentChunkFreeSpace == 0) {
			   // Move to the next chunk or creates a new chunk to store the new 
			   // element.

			   currentChunk++;
			   if (currentChunk >= chunks.size()) {
				   chunks.resize(chunks.size()+1);
			   } // end if

			   currentChunkFreeSpace = DEFAULT_CHUNK_SIZE;
		   } // end else
			
			auto &c = chunks[currentChunk];
			e = &(c.elements[DEFAULT_CHUNK_SIZE - currentChunkFreeSpace]);
			lastInsertedElementId = (currentChunk+1)*DEFAULT_CHUNK_SIZE - 
				currentChunkFreeSpace;
			
			currentChunkFreeSpace--;
		} // end else

		e->deleted = false;
		numElements++;

		return e;
	} // end method	
	
public:	
	
	List() : 
			numElements(0), 
			lastInsertedElementId(-1),
			currentChunk(-1),
			currentChunkFreeSpace(0) {
	}; // end constructor

	~List() {
		for (DestructorCallback &callback : callbackOnDestructor) {
			callback();
		} // end for		
	} // end destructor
	
	bool isEmpty() const { return !numElements; };

	int size() const { return numElements; };
	int lastId() const { return lastInsertedElementId; }
	int largestId() const { return (currentChunk+1)*DEFAULT_CHUNK_SIZE 
		- currentChunkFreeSpace; }

	int recycleId() const {
		return available.empty()? -1 : available.front();
	} // end method
	
	int capacity() const {
		// It can be optimized, but for now its just for debug purpose.
		int capacity = 0;

		for (auto &c : chunks )
			capacity += DEFAULT_CHUNK_SIZE;

		return capacity;
	} // end method

	void reserve(const int n) {
		const int numChunks = (int) std::ceil(n / double(DEFAULT_CHUNK_SIZE));
		if (numChunks > chunks.size()) {
			chunks.resize(numChunks);
		} // end if
	}; // end method

	Element<T> *create() {
		Element<T> *e = create_internal();
		for (CreateElementCallback &callback : callbackOnCreate) {
			callback(lastId());
		} // end for
		return e;
	} // end method
	
	Element<T> *add(const T &value) {
		Element<T> *e = create_internal();
		e->value = value;
		for (CreateElementCallback &callback : callbackOnCreate) {
			callback(lastId());
		} // end for
		return e;
	} // end method

	Element<T> *get(const int index) {
		return &chunks[index/DEFAULT_CHUNK_SIZE].elements[index%DEFAULT_CHUNK_SIZE];
	} // end method
	
	const Element<T> *get(const int index) const {
		return &chunks[index/DEFAULT_CHUNK_SIZE].elements[index%DEFAULT_CHUNK_SIZE];
	} // end method	
	
	void remove(const int index) {
		Element<T> *e = get(index);
		e->deleted = true;
                
		available.push_back(index);
		numElements--;
		
		for (RemoveElementCallback &callback : callbackOnRemove) {
			callback(index);
		} // end for
	} // end method
	
	DestructorCallbackHandler addDestructorEventCallback(DestructorCallback callback) {
		callbackOnDestructor.push_back(callback);
		return --callbackOnDestructor.end();
	} // end method	
	
	CreateElementCallbackHandler addCreateCallback(CreateElementCallback callback) {
		callbackOnCreate.push_back(callback);
		return --callbackOnCreate.end();
	} // end method

	RemoveElementCallbackHandler addRemoveCallback(CreateElementCallback callback) {
		callbackOnRemove.push_back(callback);
		return --callbackOnRemove.end();
	} // end method

	void deleteDestructorCallback(DestructorCallbackHandler handler) {
		callbackOnDestructor.erase(handler);
	} // end method	
	
	void deleteCreateCallback(CreateElementCallbackHandler handler) {
		callbackOnCreate.erase(handler);
	} // end method	

	void deleteRemoveCallback(RemoveElementCallbackHandler handler) {
		callbackOnRemove.erase(handler);
	} // end method	
	
	class Iterator {
	friend class List;
	private:
		List *l;
		Element<T> *e;
		int currChunk;
		int currElementInChunk;
		int size;
		bool stop;
		
		Element<T> *nextElement() {
			do {				
				currElementInChunk++;
			
				if (currElementInChunk == DEFAULT_CHUNK_SIZE) {
					currElementInChunk = 0;
					currChunk++;
					
					if (currChunk == l->chunks.size()) {
						stop = true;
						break;
					} // end if
					
					auto &chunk = l->chunks[currChunk];
					e = &(chunk.elements[0]);	
					
				} else {
					e++;
				} // end else
			} while (e->deleted);	
			
			return e;
		} // end method
		
	public:		
		Iterator() : l(NULL), e(NULL), currChunk(0), currElementInChunk(0) {};
		Iterator( List *l ) : l(l), currChunk(0), currElementInChunk(0) {
			if (!l->isEmpty()) {
				e = l->get(0);
				if (e->deleted) {
					// Skip deleted elements at the beginning of the list.
					e = nextElement();
				} // end if
				stop = false;
			} else {
				stop = true;
			} // end else
		} // end constructor
		
		void operator++ () {
			e = nextElement();
		} // end method
		
		ElementPointer<T> operator*() { return e; }
		
		friend bool operator != ( Iterator it1, Iterator it2 ) {
			return !it1.stop;
		} // end operator
		
		bool stopFlag() { return stop; }
	}; // end structure
	
	Iterator begin() {
		return Iterator( this );
	}
	
	Iterator end() {
		Iterator end;
		
		end.l = this;
		end.currChunk = chunks.size() - 1;
		end.currElementInChunk = DEFAULT_CHUNK_SIZE - 1;
		end.e = &(chunks[ end.currChunk ].elements[ end.currElementInChunk ]);
		
		return end;
	}	
	
	/* Method for debug purposes */
	void printState() {
		std::cout << " --- List state---" << std::endl;
		
		std::cout << "> Number of elements: " << numElements << std::endl;
		std::cout << "> Current chunk free elements: " << currentChunkFreeSpace << std::endl;
		std::cout << "> Number of chunks: " << chunks.size() << std::endl;
		for( auto c: chunks ) 
			std::cout << ">> Chunk size: " << c.size << std:: endl;
		
		std::cout << " -----------------" << std::endl;
	} // end method
	
}; // end class

// -----------------------------------------------------------------------------

template<typename T, unsigned int DEFAULT_CHUNK_SIZE = 1000>
class ConstList {
private:
	List<T, DEFAULT_CHUNK_SIZE> *l;
	ConstList() {}
public:
	
	typedef typename List<T, DEFAULT_CHUNK_SIZE>::Iterator Iterator;
	
	ConstList(const List<T, DEFAULT_CHUNK_SIZE> &list) {
		l = const_cast<List<T, DEFAULT_CHUNK_SIZE> *>(&list);
	} // end constructor
	
	Element<T> *get(const int index) { return l->get(index); }
	Iterator begin() { return l->begin(); }
	Iterator end() { return l->end(); }
}; // end class


} // end namespace

#endif

