/*
 *  vectorCache.h
 *  games
 *
 *  Created by Nathan Sturtevant on 8/25/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#ifndef QUEUECACHE_H
#define QUEUECACHE_H

template<class storage>
class queueCache {
public:
	queueCache() { count = 0; }
	~queueCache();
	queueCache(const queueCache<storage> &v) { count = 0; freeList.resize(0); }
	queueCache<storage> &operator=(const queueCache<storage> &v) = delete;
	std::queue<storage> *getItem();
	void returnItem(std::queue<storage> *);
private:
	std::vector<std::queue<storage> *> freeList;
	int count;
};


template<class storage>
queueCache<storage>::~queueCache<storage>()
{
//	printf("Cached storage destroyed\n");
	for (unsigned int x = 0; x < freeList.size(); x++)
		delete freeList[x];
	freeList.resize(0);
	count = 0;
}

template<class storage>
std::queue<storage> *queueCache<storage>::getItem()
{
	if (freeList.size() > 0)
	{
		std::queue<storage> *theItem = freeList.back();
		freeList.pop_back();
//		printf("CACHE: REALLOC: %p\n", theItem);
		return theItem;
	}
	else {
//		printf("%d items allocated\n", ++count);
		std::queue<storage> *newItem = new std::queue<storage>();
//		printf("CACHE: ALLOC: %p\n", newItem);
		return newItem;
//		theCache.resize(theCache.size()+1);
//		printf("CACHE: ALLOC: %p\n", &theCache[theCache.size()-1]);
//		return &theCache[theCache.size()-1];
	}
//	printf("CACHE: PTR: %p\n", (void*)0);
	return 0;
}

template<class storage>
void queueCache<storage>::returnItem(std::queue<storage> *item)
{
//	printf("CACHE: FREE: %p\n", item);
	while(!item->empty()) item->pop();
	// item->clear();
	freeList.push_back(item);
}

#endif