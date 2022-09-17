#pragma once

namespace ProjectAI
{
	class Log
	{
	public:
		static void Init();

		inline static std::shared_ptr<spdlog::logger> &GetLogger() { return s_Logger; }

	private:
		static std::shared_ptr<spdlog::logger> s_Logger;
	};
}

#define CORE_TRACE(...) ProjectAI::Log::GetLogger()->trace(__VA_ARGS__)
#define CORE_INFO(...) ProjectAI::Log::GetLogger()->info(__VA_ARGS__)
#define CORE_WARN(...) ProjectAI::Log::GetLogger()->warn(__VA_ARGS__)
#define CORE_ERROR(...) ProjectAI::Log::GetLogger()->error(__VA_ARGS__)

#define CORE_DEBUGBREAK() raise(SIGTRAP)
#define CORE_INTERNAL_ASSERT_IMPL(type, check, msg, ...) \
	if (!(check))                                        \
	{                                                    \
		CORE_ERROR(msg, __VA_ARGS__);                    \
		CORE_DEBUGBREAK();                               \
	}
#define CORE_INTERNAL_ASSERT_WITH_MSG(type, check, ...) CORE_INTERNAL_ASSERT_IMPL(type, check, "Assertion failed: {0}", __VA_ARGS__)
#define CORE_INTERNAL_ASSERT_NO_MSG(type, check) CORE_INTERNAL_ASSERT_IMPL(type, check, "Assertion '{0}' failed at {1}:{2}", #check, __FILE__, __LINE__)
#define CORE_INTERNAL_ASSERT_GET_MACRO_NAME(arg1, arg2, macro, ...) macro
#define CORE_INTERNAL_ASSERT_GET_MACRO(...) CORE_INTERNAL_ASSERT_GET_MACRO_NAME(__VA_ARGS__, CORE_INTERNAL_ASSERT_WITH_MSG, CORE_INTERNAL_ASSERT_NO_MSG)
#define CORE_ASSERT(...) CORE_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_, __VA_ARGS__)