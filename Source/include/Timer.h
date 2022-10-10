#pragma once

#include "Log.h"

namespace ProjectAI
{
	class ScopedTimer
	{
	public:
		ScopedTimer(const char *name)
		{
			m_Name = name;
			Reset();
		}
		~ScopedTimer() { CORE_WARN("{0} done in {1} milliseconds\n", m_Name, ElapsedMillis()); }

		void Reset() { m_Start = std::chrono::high_resolution_clock::now(); }

		int64_t ElapsedSeconds() { return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - m_Start).count(); }
		int64_t ElapsedMillis() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - m_Start).count(); }
		int64_t ElapsedMicros() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_Start).count(); }
		int64_t ElapsedNanos() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - m_Start).count(); }

	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> m_Start;
		const char *m_Name;
	};

	class Timestep
	{
	public:
		Timestep() { m_StartTime = std::chrono::steady_clock::now(); }

		void Update() { m_Time = std::chrono::steady_clock::now(); }
		float GetSeconds() const { return (float)std::chrono::duration_cast<std::chrono::seconds>(m_Time - m_StartTime).count(); }
		float GetMilliseconds() const { return GetSeconds() * 1000; }

		operator float() const { return GetSeconds(); }

	private:
		std::chrono::steady_clock::time_point m_Time;
		std::chrono::steady_clock::time_point m_StartTime;
	};
}