#ifndef LOGGING_H
#define LOGGING_H

#pragma once

#include <stdint.h>
#include <stdio.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#if defined(_WIN32)
#define BN_LOG(msg, ...) do { char buffer[2048]; snprintf(buffer, sizeof(buffer), "[%s:%d]: " msg "\n", __FILE__, __LINE__, ## __VA_ARGS__); OutputDebugStringA(buffer); } while(0)
#else
#define BN_LOG(msg, ...) printf("[%s:%d]: " msg "\n", __FILE__, __LINE__, ## __VA_ARGS__)
#endif

#define BNS_PROFILE_SCOPE(name) __ScopeProfiler __scoped_Profiler(name)
//#define BNS_PROFILE_SCOPE(name) 

struct __ScopeProfiler {
#if defined(_WIN32)
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
#else
	__ScopeProfiler(const char* _name) {
		// TODO	
	}
#endif
};


#endif
