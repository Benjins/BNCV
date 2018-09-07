#ifndef LOGGING_H
#define LOGGING_H

#pragma once

#include <stdint.h>
#include <stdio.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#define BN_LOG(msg, ...) printf("[%s:%d]: " msg "\n", __FILE__, __LINE__, ## __VA_ARGS__)

#define BNS_PROFILE_SCOPE(name) __ScopeProfiler __scoped_Profiler(name)
//#define BNS_PROFILE_SCOPE(name) 

struct __ScopeProfiler {
	LARGE_INTEGER start;
	const char* name;

	__ScopeProfiler(const char* _name) {
		name = _name;
		QueryPerformanceCounter(&start);
	}

	~__ScopeProfiler() {
		LARGE_INTEGER end;
		QueryPerformanceCounter(&end);
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);

		uint64_t usec = (end.QuadPart - start.QuadPart) * 1000 * 1000 / freq.QuadPart;
		BN_LOG("'%s' scope elapsed with %llu usec", name, usec);
	}
};


#endif